//! Headless GigE end-to-end streamer / debugger.
//!
//! Opens a camera by IP, fetches+inflates the (possibly zipped) GenICam XML,
//! configures mono full-frame acquisition, issues AcquisitionStart, and reports
//! exactly what arrives on the GVSP stream socket — packet counts, leader
//! geometry, frame completion, pixel min/max. This mirrors the viewer's
//! `gev_camera` pipeline but with verbose diagnostics so we can see where
//! streaming breaks.
//!
//!   cargo run --example gev_stream --features gev -- 192.168.0.2
//!
//! Uses a plain (sync) main + manual current-thread runtime so the synchronous
//! cameleon-genapi `Device` bridge can `block_on` without nesting.

use std::io::Read;
use std::net::{IpAddr, Ipv4Addr, SocketAddr};
use std::time::{Duration, Instant};

use bytes::BytesMut;
use cameleon_genapi::store::{DefaultCacheStore, DefaultNodeStore, DefaultValueStore, NodeStore};
use cameleon_genapi::interface::{ICommand, IEnumeration, IFloat, IInteger};
use cameleon_genapi::ValueCtxt;
use tokio::runtime::Runtime;
use viva_gige::gvcp::{self, GigeDevice};
use viva_gige::gvsp::{self, FrameAssembly, GvspPacket};
use viva_gige::nic::{self, Iface};

struct GenApi {
    store: DefaultNodeStore,
    ctxt: ValueCtxt<DefaultValueStore, DefaultCacheStore>,
}

struct Bridge<'a> {
    rt: &'a Runtime,
    dev: &'a mut GigeDevice,
}
impl cameleon_genapi::Device for Bridge<'_> {
    fn read_mem(&mut self, address: i64, buf: &mut [u8]) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let data = self.rt.block_on(self.dev.read_mem(address as u64, buf.len()))?;
        let n = buf.len().min(data.len());
        buf[..n].copy_from_slice(&data[..n]);
        Ok(())
    }
    fn write_mem(&mut self, address: i64, data: &[u8]) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        self.rt.block_on(self.dev.write_mem(address as u64, data))?;
        Ok(())
    }
}

fn main() -> anyhow::Result<()> {
    let ip: Ipv4Addr = std::env::args().nth(1).unwrap_or_else(|| "192.168.0.2".into()).parse()?;
    let rt = tokio::runtime::Builder::new_current_thread().enable_all().build()?;

    println!("Opening {ip}…");
    let mut dev = rt.block_on(GigeDevice::open(SocketAddr::new(IpAddr::V4(ip), gvcp::GVCP_PORT)))?;
    rt.block_on(dev.claim_control())?;
    println!("control claimed");

    let mut g = load_genapi(&rt, &mut dev)?;
    println!("GenICam parsed");

    // Configure full-frame mono. Print each step.
    {
        let mut b = Bridge { rt: &rt, dev: &mut dev };
        for dim in ["Width", "Height"] {
            if let Some(nid) = g.store.id_by_name(dim) {
                if let Some(i) = nid.as_iinteger_kind(&g.store) {
                    let max = i.max(&mut b, &g.store, &mut g.ctxt).unwrap_or(0);
                    let r = i.set_value(max, &mut b, &g.store, &mut g.ctxt);
                    println!("set {dim} = {max} -> {:?}", r.map(|_| "ok"));
                }
            }
        }
        if let Some(nid) = g.store.id_by_name("PixelFormat") {
            if let Some(en) = nid.as_ienumeration_kind(&g.store) {
                for want in ["Mono8", "Mono16", "Mono12", "Mono12Packed", "Mono10"] {
                    if en.entry_by_symbolic(want, &g.store).is_some() {
                        let r = en.set_entry_by_symbolic(want, &mut b, &g.store, &mut g.ctxt);
                        println!("set PixelFormat = {want} -> {:?}", r.map(|_| "ok"));
                        break;
                    }
                }
            }
        }
        if let Some(nid) = g.store.id_by_name("AcquisitionMode") {
            if let Some(en) = nid.as_ienumeration_kind(&g.store) {
                let r = en.set_entry_by_symbolic("Continuous", &mut b, &g.store, &mut g.ctxt);
                println!("set AcquisitionMode = Continuous -> {:?}", r.map(|_| "ok"));
            }
        }
        // Free-run: ensure the camera isn't waiting for a hardware/software trigger.
        if let Some(nid) = g.store.id_by_name("TriggerMode") {
            if let Some(en) = nid.as_ienumeration_kind(&g.store) {
                let r = en.set_entry_by_symbolic("Off", &mut b, &g.store, &mut g.ctxt);
                println!("set TriggerMode = Off -> {:?}", r.map(|_| "ok"));
            }
        }
        // Report the geometry we'll expect.
        for f in ["Width", "Height", "PixelFormat"] {
            if let Some(nid) = g.store.id_by_name(f) {
                if let Some(i) = nid.as_iinteger_kind(&g.store) {
                    println!("  {f} now = {:?}", i.value(&mut b, &g.store, &mut g.ctxt));
                } else if let Some(en) = nid.as_ienumeration_kind(&g.store) {
                    println!("  {f} now = {:?}", en.current_value(&mut b, &g.store, &mut g.ctxt));
                }
            }
        }
    }

    // Bind GVSP socket, then negotiate stream toward it.
    let iface = Iface::from_ipv4(local_ipv4_towards(ip)?)?;
    // Bind to 0.0.0.0 so we receive the unicast GVSP regardless of which local
    // address the OS associates it with.
    let bind_ip = IpAddr::V4(Ipv4Addr::UNSPECIFIED);
    let socket = rt.block_on(nic::bind_udp(bind_ip, 0, Some(iface.clone()), Some(32 * 1024 * 1024)))?;
    let local_port = socket.local_addr()?.port();
    let params = rt.block_on(dev.negotiate_stream(0, &iface, local_port, None))?;
    let packet_payload = params.packet_size.saturating_sub(8).max(1) as usize;
    println!(
        "stream negotiated: host={} port={} mtu={} packet_size={} (payload={})",
        params.host, local_port, params.mtu, params.packet_size, packet_payload
    );
    // Read back the stream-channel registers to confirm the camera accepted them.
    for (name, addr) in [("SCPHostPort", 0x0D00u32), ("SCPSPacketSize", 0x0D04), ("SCPD", 0x0D08), ("SCDA", 0x0D18)] {
        match rt.block_on(dev.read_register(addr)) {
            Ok(v) => {
                let extra = if name == "SCDA" { format!(" = {}", Ipv4Addr::from(v)) } else { String::new() };
                println!("  reg {name} @0x{addr:04x} = 0x{v:08x} ({v}){extra}");
            }
            Err(e) => println!("  reg {name} @0x{addr:04x} read err: {e}"),
        }
    }

    // AcquisitionStart.
    {
        let mut b = Bridge { rt: &rt, dev: &mut dev };
        // Lock transport-layer params — FLIR/Point Grey won't stream without this.
        if let Some(nid) = g.store.id_by_name("TLParamsLocked") {
            if let Some(i) = nid.as_iinteger_kind(&g.store) {
                let r = i.set_value(1, &mut b, &g.store, &mut g.ctxt);
                println!("set TLParamsLocked = 1 -> {:?}", r.map(|_| "ok"));
            }
        } else {
            println!("(no TLParamsLocked node)");
        }
        if let Some(nid) = g.store.id_by_name("AcquisitionStart") {
            if let Some(c) = nid.as_icommand_kind(&g.store) {
                let r = c.execute(&mut b, &g.store, &mut g.ctxt);
                println!("AcquisitionStart -> {:?}", r.map(|_| "ok"));
            } else {
                println!("AcquisitionStart node is not a command?!");
            }
        } else {
            println!("no AcquisitionStart node found");
        }
    }

    // Receive window (seconds) — optional 2nd arg, default 8.
    let listen_secs: u64 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(8);
    println!("\nlistening for GVSP packets for {listen_secs} s…");
    let mut buf = vec![0u8; 65536];
    let mut assembly: Option<FrameAssembly> = None;
    let mut geom: Option<(u32, u32, u32)> = None;
    let mut pkts = 0u64;
    let (mut leaders, mut payloads, mut trailers, mut frames) = (0u64, 0u64, 0u64, 0u64);
    let mut first_pkt_ids: Vec<u32> = Vec::new();
    let deadline = Instant::now() + Duration::from_secs(listen_secs);

    // Hole-punch: the host's network-extension filter allows inbound that
    // corresponds to an outbound flow (that's why GVCP replies get through). The
    // camera streams from a varying low source port (observed 1051, 1054…), so we
    // spray a punch across the whole likely range from our receive socket; one of
    // them matches the camera's actual source port and opens the allowed flow.
    let mut known_port: Option<u16> = None;
    rt.block_on(async {
        for p in 1024u16..=2048 {
            let _ = socket.send_to(&[0u8], SocketAddr::new(IpAddr::V4(ip), p)).await;
        }
        println!("(sprayed hole-punch to camera ports 1024-2048)");

        let deadline_t = tokio::time::Instant::now() + Duration::from_secs(listen_secs);
        let sleep = tokio::time::sleep_until(deadline_t);
        tokio::pin!(sleep);
        let mut punch_timer = tokio::time::interval(Duration::from_millis(20));
        punch_timer.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);

        loop {
            // biased: drain the socket before punching, so high packet rates aren't
            // throttled by per-iteration timer overhead. Punch only fires when recv
            // has nothing ready (i.e. the flow stalled and needs reopening).
            tokio::select! {
                biased;
                _ = &mut sleep => break,
                r = socket.recv_from(&mut buf) => {
                    let (n, src) = match r { Ok(v) => v, Err(e) => { println!("recv error: {e}"); break; } };
                    if known_port.is_none() {
                        known_port = Some(src.port());
                        println!("(camera GVSP source port = {})", src.port());
                    }
                    pkts += 1;
                    match gvsp::parse_packet(&buf[..n]) {
                        Ok(GvspPacket::Leader { block_id, width, height, pixel_format, packet_id, .. }) => {
                            leaders += 1;
                            if leaders <= 3 {
                                println!("  LEADER block={block_id} pkt_id={packet_id} {width}x{height} pf=0x{pixel_format:08x}");
                            }
                            geom = Some((width, height, pixel_format));
                            let bpp = bytes_per_pixel(pixel_format).max(1);
                            let total = width as usize * height as usize * bpp;
                            let expected = total.div_ceil(packet_payload).max(1);
                            let pool = BytesMut::zeroed(expected * packet_payload);
                            assembly = Some(FrameAssembly::new(block_id, expected, packet_payload, pool, Instant::now() + Duration::from_secs(2)));
                        }
                        Ok(GvspPacket::Payload { block_id, packet_id, data }) => {
                            payloads += 1;
                            if first_pkt_ids.len() < 6 { first_pkt_ids.push(packet_id); }
                            if let Some(a) = assembly.as_mut() {
                                if a.block_id() == block_id {
                                    a.ingest(packet_id.saturating_sub(1) as usize, &data);
                                }
                            }
                        }
                        Ok(GvspPacket::Trailer { block_id, .. }) => {
                            trailers += 1;
                            if let Some(a) = assembly.as_ref() {
                                if a.block_id() == block_id {
                                    let a = assembly.take().unwrap();
                                    match (a.finish(), geom) {
                                        (Some(payload), Some((w, h, pf))) => {
                                            frames += 1;
                                            if frames <= 5 { report_frame(&payload, w, h, pf); }
                                        }
                                        (None, _) => if trailers <= 3 { println!("  TRAILER block={block_id}: frame INCOMPLETE (missing packets)"); },
                                        _ => {}
                                    }
                                }
                            }
                        }
                        Err(e) => if pkts <= 5 { println!("  parse error: {e:?} (len {n})"); },
                    }
                }
                _ = punch_timer.tick() => {
                    match known_port {
                        Some(p) => { let _ = socket.send_to(&[0u8], SocketAddr::new(IpAddr::V4(ip), p)).await; }
                        None => for p in 1024u16..=2048 {
                            let _ = socket.send_to(&[0u8], SocketAddr::new(IpAddr::V4(ip), p)).await;
                        }
                    }
                }
            }
        }
    });

    println!("\n── summary ──");
    println!("packets={pkts}  leaders={leaders}  payloads={payloads}  trailers={trailers}  complete_frames={frames}");
    if !first_pkt_ids.is_empty() {
        println!("first payload packet_ids seen: {first_pkt_ids:?}");
    }
    if pkts == 0 {
        println!("NO GVSP packets arrived. Likely: AcquisitionStart failed, or macOS firewall is\n\
                  dropping inbound UDP to this binary, or the stream destination didn't take.");
    }

    let mut b = Bridge { rt: &rt, dev: &mut dev };
    if let Some(nid) = g.store.id_by_name("AcquisitionStop") {
        if let Some(c) = nid.as_icommand_kind(&g.store) {
            let _ = c.execute(&mut b, &g.store, &mut g.ctxt);
        }
    }
    let _ = rt.block_on(dev.release_control());
    Ok(())
}

fn report_frame(payload: &[u8], w: u32, h: u32, pf: u32) {
    let npix = w as usize * h as usize;
    let (min, max) = match pf {
        0x01080001 => minmax(payload.iter().take(npix).map(|&v| v as u32)),
        _ if payload.len() >= npix * 2 => minmax(payload[..npix * 2].chunks_exact(2).map(|c| u16::from_le_bytes([c[0], c[1]]) as u32)),
        _ => (0, 0),
    };
    println!("  ✓ FRAME {w}x{h} pf=0x{pf:08x} payload={}B  min={min} max={max}", payload.len());
}

fn minmax(it: impl Iterator<Item = u32>) -> (u32, u32) {
    it.fold((u32::MAX, 0), |(lo, hi), v| (lo.min(v), hi.max(v)))
}

fn bytes_per_pixel(pf: u32) -> usize {
    match pf {
        0x01080001 => 1,
        0x010C0047 | 0x010A0046 => 0, // packed
        _ => 2,
    }
}

fn load_genapi(rt: &Runtime, dev: &mut GigeDevice) -> anyhow::Result<GenApi> {
    let raw = rt.block_on(dev.read_mem(0x0200, 512))?;
    let url = String::from_utf8_lossy(&raw);
    let url = url.trim_end_matches(['\0', ' ', '\r', '\n']).trim();
    println!("GenICam URL: {url}");
    let rest = url.strip_prefix("Local:").or_else(|| url.strip_prefix("local:"))
        .ok_or_else(|| anyhow::anyhow!("unsupported URL scheme: {url}"))?;
    let mut parts = rest.split(';');
    let filename = parts.next().unwrap_or_default().to_string();
    let addr = u64::from_str_radix(parts.next().unwrap_or("0").trim_start_matches("0x"), 16)?;
    let len = usize::from_str_radix(parts.next().unwrap_or("0").trim_start_matches("0x"), 16)?;

    let mut bytes = Vec::with_capacity(len);
    let mut off = 0;
    while off < len {
        let this = 512.min(len - off);
        bytes.extend_from_slice(&rt.block_on(dev.read_mem(addr + off as u64, this))?);
        off += this;
    }

    let xml = if filename.to_ascii_lowercase().ends_with(".zip") {
        let mut archive = zip::ZipArchive::new(std::io::Cursor::new(bytes))?;
        let mut s = String::new();
        for i in 0..archive.len() {
            let mut e = archive.by_index(i)?;
            if e.name().to_ascii_lowercase().ends_with(".xml") { e.read_to_string(&mut s)?; break; }
        }
        s
    } else {
        String::from_utf8(bytes)?
    };

    let (_rd, store, ctxt) = cameleon_genapi::builder::GenApiBuilder::<DefaultNodeStore, DefaultValueStore, DefaultCacheStore>::default()
        .build(&xml)
        .map_err(|e| anyhow::anyhow!("genapi parse: {e}"))?;
    Ok(GenApi { store, ctxt })
}

fn local_ipv4_towards(target: Ipv4Addr) -> anyhow::Result<Ipv4Addr> {
    use std::net::UdpSocket;
    let sock = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, 0))?;
    sock.connect((target, gvcp::GVCP_PORT))?;
    match sock.local_addr()? {
        SocketAddr::V4(a) => Ok(*a.ip()),
        _ => Ok(Ipv4Addr::UNSPECIFIED),
    }
}
