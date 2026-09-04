//! Close a camera's stream channel the way eBUS and aravis do at stop:
//! AcquisitionStop, TLParamsLocked=0, then zero GevSCPHostPort and GevSCDA
//! before releasing control. Diagnostic for a camera that will not emit
//! after a previous session stopped it. Usage: gev_close [ip]

#[path = "../src/gige/mod.rs"]
#[allow(dead_code)]
mod gige;

use std::io::Read;
use std::net::{IpAddr, Ipv4Addr, SocketAddr};

use cameleon_genapi::interface::{ICommand, IInteger};
use cameleon_genapi::store::{DefaultCacheStore, DefaultNodeStore, DefaultValueStore, NodeStore};
use cameleon_genapi::ValueCtxt;

use gige::gvcp::{self, Device};

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
    let mut dev = Device::open(SocketAddr::new(IpAddr::V4(ip), gvcp::GVCP_PORT))
        .map_err(|e| anyhow::anyhow!("open: {e}"))?;
    dev.claim_control().map_err(|e| anyhow::anyhow!("claim control: {e}"))?;
    println!("control claimed");
    let mut g = load_genapi(&mut dev)?;
    {
        let mut b = Bridge { dev: &mut dev };
        if let Some(nid) = g.store.id_by_name("AcquisitionStop") {
            if let Some(c) = nid.as_icommand_kind(&g.store) {
                println!("AcquisitionStop -> {:?}", c.execute(&mut b, &g.store, &mut g.ctxt).map(|_| "ok"));
            }
        }
        if let Some(nid) = g.store.id_by_name("TLParamsLocked") {
            if let Some(i) = nid.as_iinteger_kind(&g.store) {
                println!("TLParamsLocked = 0 -> {:?}", i.set_value(0, &mut b, &g.store, &mut g.ctxt).map(|_| "ok"));
            }
        }
    }
    // Close the stream channel: no host port, no destination.
    println!("GevSCPHostPort = 0 -> {:?}", dev.write_register(0x0d00, 0).map(|_| "ok"));
    println!("GevSCDA = 0 -> {:?}", dev.write_register(0x0d18, 0).map(|_| "ok"));
    println!("release control -> {:?}", dev.release_control().map(|_| "ok"));
    Ok(())
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
