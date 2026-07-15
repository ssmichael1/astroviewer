//! Reboot a GigE camera via its GenICam DeviceReset command, restoring
//! power-up defaults (useful after setting a mode that wedges the camera).
//!
//!   cargo run --example gev_reset --features gev -- 192.168.0.2
//!
//! Finds the camera's reset command node (DeviceReset, or any *Reset command
//! as a fallback), executes it, then polls until the camera answers GVCP
//! again. The camera must have a persistent/forced IP to come back at the
//! same address.

use std::io::Read;
use std::net::{IpAddr, Ipv4Addr, SocketAddr};
use std::time::{Duration, Instant};

use cameleon_genapi::store::{DefaultCacheStore, DefaultNodeStore, DefaultValueStore, NodeStore};
use cameleon_genapi::interface::ICommand;
use cameleon_genapi::ValueCtxt;
use tokio::runtime::Runtime;
use viva_gige::gvcp::{self, GigeDevice};

struct Bridge<'a> { rt: &'a Runtime, dev: &'a mut GigeDevice }
impl cameleon_genapi::Device for Bridge<'_> {
    fn read_mem(&mut self, a: i64, buf: &mut [u8]) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let d = self.rt.block_on(self.dev.read_mem(a as u64, buf.len()))?;
        let n = buf.len().min(d.len()); buf[..n].copy_from_slice(&d[..n]); Ok(())
    }
    fn write_mem(&mut self, a: i64, d: &[u8]) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        self.rt.block_on(self.dev.write_mem(a as u64, d))?; Ok(())
    }
}

fn main() -> anyhow::Result<()> {
    let _ = tracing_subscriber::fmt().with_max_level(tracing::Level::INFO).try_init();
    let ip: Ipv4Addr = std::env::args().nth(1).unwrap_or_else(|| "192.168.0.2".into()).parse()?;
    let addr = SocketAddr::new(IpAddr::V4(ip), gvcp::GVCP_PORT);
    let rt = tokio::runtime::Builder::new_current_thread().enable_all().build()?;

    println!("Opening {ip}…");
    let mut dev = rt.block_on(GigeDevice::open(addr))?;
    rt.block_on(dev.claim_control())?;

    let (store, mut ctxt) = load_genapi(&rt, &mut dev)?;

    // The standard SFNC name plus vendor variants.
    let nid = ["DeviceReset", "CameraReset", "DeviceSoftReset", "SoftReset", "Reset"]
        .iter()
        .find_map(|n| store.id_by_name(n))
        .ok_or_else(|| anyhow::anyhow!("camera exposes no DeviceReset-like command"))?;
    let name = nid.name(&store).to_string();
    let cmd = nid.as_icommand_kind(&store)
        .ok_or_else(|| anyhow::anyhow!("{name} is not a command node"))?;

    println!("Executing {name}…");
    let mut bridge = Bridge { rt: &rt, dev: &mut dev };
    // The camera typically reboots before acking the register write, so a
    // timeout here is expected — report it and continue to the reachability poll.
    match cmd.execute(&mut bridge, &store, &mut ctxt) {
        Ok(()) => println!("{name} acknowledged."),
        Err(e) => println!("{name} sent (no ack: {e}); camera is likely rebooting."),
    }
    drop(dev); // old control channel is dead across the reboot

    println!("Waiting for the camera to come back at {ip}…");
    std::thread::sleep(Duration::from_secs(3));
    let deadline = Instant::now() + Duration::from_secs(60);
    loop {
        match rt.block_on(GigeDevice::open(addr)) {
            Ok(mut d) => {
                let _ = rt.block_on(d.release_control());
                println!("✓ camera answering at {ip} again. Power-up defaults restored.");
                return Ok(());
            }
            Err(e) if Instant::now() < deadline => {
                println!("  …not back yet ({e})");
                std::thread::sleep(Duration::from_secs(2));
            }
            Err(e) => anyhow::bail!("camera never came back at {ip}: {e} — power-cycle it manually"),
        }
    }
}

fn load_genapi(rt: &Runtime, dev: &mut GigeDevice)
    -> anyhow::Result<(DefaultNodeStore, ValueCtxt<DefaultValueStore, DefaultCacheStore>)>
{
    let raw = rt.block_on(dev.read_mem(0x0200, 512))?;
    let end = raw.iter().position(|&b| b == 0).unwrap_or(raw.len()); // cut at NUL; tail may be garbage
    let url = String::from_utf8_lossy(&raw[..end]);
    let url = url.trim();
    let url = url.split('?').next().unwrap_or(url); // drop "?SchemaVersion=…"
    let rest = url.strip_prefix("Local:").or_else(|| url.strip_prefix("local:"))
        .ok_or_else(|| anyhow::anyhow!("unsupported URL: {url}"))?;
    let mut parts = rest.split(';');
    let filename = parts.next().unwrap_or_default().trim().to_string();
    let addr = u64::from_str_radix(parts.next().unwrap_or("0").trim().trim_start_matches("0x"), 16)?;
    let len = usize::from_str_radix(parts.next().unwrap_or("0").trim().trim_start_matches("0x"), 16)?;
    let mut bytes = Vec::with_capacity(len);
    let mut off = 0;
    while off < len {
        let want = 512.min(len - off);
        let req = (want + 3) & !3; // GVCP READMEM count must be 4-byte aligned
        let part = rt.block_on(dev.read_mem(addr + off as u64, req))?;
        bytes.extend_from_slice(&part[..want.min(part.len())]);
        off += want;
    }
    let xml = if filename.to_ascii_lowercase().ends_with(".zip") {
        let mut a = zip::ZipArchive::new(std::io::Cursor::new(bytes))?;
        let mut s = String::new();
        for i in 0..a.len() {
            let mut e = a.by_index(i)?;
            if e.name().to_ascii_lowercase().ends_with(".xml") { e.read_to_string(&mut s)?; break; }
        }
        s
    } else { String::from_utf8(bytes)? };
    let (_rd, store, ctxt) = cameleon_genapi::builder::GenApiBuilder::<DefaultNodeStore, DefaultValueStore, DefaultCacheStore>::default()
        .build(&xml).map_err(|e| anyhow::anyhow!("genapi parse: {e}"))?;
    Ok((store, ctxt))
}
