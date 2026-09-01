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
//!   cargo run --example gev_stream --features gev -- 127.0.0.1 8 3957
//!
//! Args: <ip> [listen_secs=8] [gvcp_port=3956]. Uses the app-owned synchronous
//! GigE transport (no async runtime).

#[path = "../src/gige/mod.rs"]
#[allow(dead_code)]
mod gige;

use std::io::Read;
use std::net::{IpAddr, Ipv4Addr, SocketAddr};
use std::time::{Duration, Instant};

use cameleon_genapi::interface::{ICommand, IEnumeration, IInteger};
use cameleon_genapi::store::{DefaultCacheStore, DefaultNodeStore, DefaultValueStore, NodeStore};
use cameleon_genapi::ValueCtxt;

use gige::gvcp::{self, Device};
use gige::gvsp::{self, FrameAssembly, GvspPacket};
use gige::nic::{self, Iface};

struct GenApi {
    store: DefaultNodeStore,
    ctxt: ValueCtxt<DefaultValueStore, DefaultCacheStore>,
}

struct Bridge<'a> {
    dev: &'a mut Device,
}
impl cameleon_genapi::Device for Bridge<'_> {
    fn read_mem(&mut self, address: i64, buf: &mut [u8]) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let data = self.dev.read_mem(address as u64, buf.len())?;
        let n = buf.len().min(data.len());
        buf[..n].copy_from_slice(&data[..n]);
        Ok(())
    }
    fn write_mem(&mut self, address: i64, data: &[u8]) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        self.dev.write_mem(address as u64, data)?;
        Ok(())
    }
}

fn main() -> anyhow::Result<()> {
    let ip: Ipv4Addr = std::env::args().nth(1).unwrap_or_else(|| "192.168.0.2".into()).parse()?;
    let listen_secs: u64 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(8);
    let port: u16 = std::env::args().nth(3).and_then(|s| s.parse().ok()).unwrap_or(gvcp::GVCP_PORT);

    println!("Opening {ip}:{port}…");
    let mut dev = Device::open(SocketAddr::new(IpAddr::V4(ip), port))
        .map_err(|e| anyhow::anyhow!("open: {e}"))?;
    dev.claim_control().map_err(|e| anyhow::anyhow!("claim control: {e}"))?;
    println!("control claimed");

    let mut g = load_genapi(&mut dev)?;
    println!("GenICam parsed");

    // Configure full-frame mono. Print each step.
    {
        let mut b = Bridge { dev: &mut dev };
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
                // GEV_PIXEL_FORMAT overrides; simulators emit only 8-bit payloads.
                let forced = std::env::var("GEV_PIXEL_FORMAT").ok();
                let prefs = ["Mono8", "Mono16", "Mono12", "Mono12Packed", "Mono10"];
                for want in forced.iter().map(String::as_str).chain(prefs) {
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
        if let Some(nid) = g.store.id_by_name("TriggerMode") {
            if let Some(en) = nid.as_ienumeration_kind(&g.store) {
                let r = en.set_entry_by_symbolic("Off", &mut b, &g.store, &mut g.ctxt);
                println!("set TriggerMode = Off -> {:?}", r.map(|_| "ok"));
            }
        }
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

    // Bind the GVSP socket, then negotiate the stream toward it.
    let iface = Iface::from_ipv4(nic::local_ipv4_towards(ip, port))?;
    let bind_ip = iface.ipv4().map(IpAddr::V4).unwrap_or(IpAddr::V4(Ipv4Addr::UNSPECIFIED));
    let (socket, recv_buffer) = nic::bind_gvsp_socket(bind_ip, 32 << 20, Duration::from_millis(200))?;
    let local_port = socket.local_addr()?.port();
    let cap: Option<u32> = std::env::var("GEV_PACKET_SIZE").ok().and_then(|s| s.parse().ok());
    let params = dev.negotiate_stream(0, &iface, local_port, cap).map_err(|e| anyhow::anyhow!("negotiate: {e}"))?;
    let effective = dev.get_stream_packet_size(0).unwrap_or(params.packet_size);
    // GevSCPSPacketSize is the full IP datagram: image bytes per packet exclude
    // IPv4 (20) + UDP (8) + GVSP (8) = 36 bytes of headers.
    let mut stride = effective.saturating_sub(36).max(1) as usize;
    println!(
        "stream negotiated: host={} port={} mtu={} packet_size={} requested / {} effective (stride={}, {} MiB rcvbuf)",
        params.host, local_port, params.mtu, params.packet_size, effective, stride, recv_buffer >> 20
    );
    for (name, addr) in [("SCPHostPort", 0x0D00u32), ("SCPSPacketSize", 0x0D04), ("SCPD", 0x0D08), ("SCDA", 0x0D18)] {
        match dev.read_register(addr) {
            Ok(v) => {
                let extra = if name == "SCDA" { format!(" = {}", Ipv4Addr::from(v)) } else { String::new() };
                println!("  reg {name} @0x{addr:04x} = 0x{v:08x} ({v}){extra}");
            }
            Err(e) => println!("  reg {name} @0x{addr:04x} read err: {e}"),
        }
    }

    // AcquisitionStart.
    {
        let mut b = Bridge { dev: &mut dev };
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
            }
        } else {
            println!("no AcquisitionStart node found");
        }
    }

    // Hole-punch the camera's GVSP source port (GevSCSP 0x0D1C), else spray.
    let known_port: Option<u16> = match dev.read_register(0x0D1C) {
        Ok(v) if v & 0xFFFF != 0 => Some((v & 0xFFFF) as u16),
        _ => None,
    };
    println!("GevSCSP (camera stream source port) = {known_port:?}");
    match known_port {
        Some(p) => { let _ = socket.send_to(&[0u8], SocketAddr::new(IpAddr::V4(ip), p)); }
        None => {
            for p in 1024u16..=2048 {
                let _ = socket.send_to(&[0u8], SocketAddr::new(IpAddr::V4(ip), p));
            }
            println!("(sprayed hole-punch to camera ports 1024-2048)");
        }
    }

    println!("\nlistening for GVSP packets for {listen_secs} s…");
    let mut buf = vec![0u8; 65536];
    let mut assembly: Option<FrameAssembly> = None;
    let mut geom: Option<(u32, u32, u32)> = None;
    let (mut pkts, mut leaders, mut payloads, mut trailers, mut frames) = (0u64, 0u64, 0u64, 0u64, 0u64);
    let mut first_pkt_ids: Vec<u32> = Vec::new();
    let mut missing_snapshot: Option<Vec<std::ops::RangeInclusive<u32>>> = None;
    let deadline = Instant::now() + Duration::from_secs(listen_secs);

    while Instant::now() < deadline {
        let (n, _src) = match socket.recv_from(&mut buf) {
            Ok(v) => v,
            Err(e) => match e.kind() {
                std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut => continue,
                std::io::ErrorKind::ConnectionReset | std::io::ErrorKind::Interrupted => continue,
                _ => { println!("recv error: {e}"); break; }
            },
        };
        pkts += 1;
        match gvsp::parse_packet(&buf[..n]) {
            Ok(GvspPacket::Leader { block_id, width, height, pixel_format }) => {
                leaders += 1;
                if leaders <= 3 {
                    println!("  LEADER block={block_id} {width}x{height} pf=0x{pixel_format:08x}");
                }
                geom = Some((width, height, pixel_format));
                let bpp = bytes_per_pixel(pixel_format).max(1);
                let total = width as usize * height as usize * bpp;
                let expected = total.div_ceil(stride).max(1);
                assembly = Some(FrameAssembly::new(block_id, expected, stride, Instant::now() + Duration::from_secs(2)));
            }
            Ok(GvspPacket::Payload { block_id, packet_id, data }) => {
                payloads += 1;
                if first_pkt_ids.len() < 6 { first_pkt_ids.push(packet_id); }
                if let (Some(a), Some((w, h, pf))) = (assembly.as_ref(), geom) {
                    let total = w as usize * h as usize * bytes_per_pixel(pf).max(1);
                    if a.block_id() == block_id && data.len() != stride && data.len() < total
                        && (packet_id == 1 || data.len() > stride)
                    {
                        println!("  !! stride corrected {stride} -> {} B/packet", data.len());
                        stride = data.len();
                        let expected = total.div_ceil(stride).max(1);
                        assembly = Some(FrameAssembly::new(block_id, expected, stride, Instant::now() + Duration::from_secs(2)));
                    }
                }
                if let Some(a) = assembly.as_mut() {
                    if a.block_id() == block_id {
                        a.ingest(packet_id.saturating_sub(1) as usize, data);
                    }
                }
            }
            Ok(GvspPacket::Trailer { block_id }) => {
                trailers += 1;
                if assembly.as_ref().map(FrameAssembly::block_id) == Some(block_id) {
                    let a = assembly.take().unwrap();
                    missing_snapshot = if a.is_complete() { None } else { Some(a.missing_ranges()) };
                    match (a.finish(), geom) {
                        (Some(payload), Some((w, h, pf))) => {
                            frames += 1;
                            if frames <= 5 { report_frame(&payload, w, h, pf); }
                        }
                        (None, _) => if trailers <= 3 {
                            println!("  TRAILER block={block_id}: frame INCOMPLETE (missing packets)");
                            if let Some(a) = missing_snapshot.take() {
                                let n: usize = a.iter().map(|r| (*r.end() - *r.start() + 1) as usize).sum();
                                let show: Vec<String> = a.iter().take(12).map(|r| format!("{}-{}", r.start() + 1, r.end() + 1)).collect();
                                println!("    missing {n} packets in {} ranges: {}{}", a.len(), show.join(" "), if a.len() > 12 { " …" } else { "" });
                            }
                        },
                        _ => {}
                    }
                }
            }
            Err(e) => if pkts <= 5 { println!("  parse error: {e:?} (len {n})"); },
        }
    }

    println!("\n── summary ──");
    println!("packets={pkts}  leaders={leaders}  payloads={payloads}  trailers={trailers}  complete_frames={frames}");
    if !first_pkt_ids.is_empty() {
        println!("first payload packet_ids seen: {first_pkt_ids:?}");
    }
    if pkts == 0 {
        println!("NO GVSP packets arrived. Likely: AcquisitionStart failed, or a host firewall is\n\
                  dropping inbound UDP to this binary, or the stream destination didn't take.");
    }

    let mut b = Bridge { dev: &mut dev };
    if let Some(nid) = g.store.id_by_name("AcquisitionStop") {
        if let Some(c) = nid.as_icommand_kind(&g.store) {
            let _ = c.execute(&mut b, &g.store, &mut g.ctxt);
        }
    }
    let _ = b.dev.release_control();
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
        0x010C0047 | 0x010A0046 | 0x010C0006 => 0, // packed
        _ => 2,
    }
}

fn load_genapi(dev: &mut Device) -> anyhow::Result<GenApi> {
    let raw = dev.read_mem(0x0200, 512).map_err(|e| anyhow::anyhow!("READMEM First-URL: {e}"))?;
    let end = raw.iter().position(|&b| b == 0).unwrap_or(raw.len());
    let url = String::from_utf8_lossy(&raw[..end]);
    let url = url.trim();
    println!("GenICam URL: {url}");
    let url = url.split('?').next().unwrap_or(url);
    let rest = url.strip_prefix("Local:").or_else(|| url.strip_prefix("local:"))
        .ok_or_else(|| anyhow::anyhow!("unsupported URL scheme: {url}"))?;
    let mut parts = rest.split(';');
    let filename = parts.next().unwrap_or_default().trim().to_string();
    let addr = u64::from_str_radix(parts.next().unwrap_or("0").trim().trim_start_matches("0x"), 16)?;
    let len = usize::from_str_radix(parts.next().unwrap_or("0").trim().trim_start_matches("0x"), 16)?;

    let mut bytes = Vec::with_capacity(len);
    let mut off = 0;
    while off < len {
        let want = 512.min(len - off);
        let req = (want + 3) & !3;
        let part = dev.read_mem(addr + off as u64, req).map_err(|e| anyhow::anyhow!("READMEM: {e}"))?;
        bytes.extend_from_slice(&part[..want.min(part.len())]);
        off += want;
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
