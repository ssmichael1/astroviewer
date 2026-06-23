//! Dump a GigE camera's full GenICam feature tree — names, types, current
//! values, ranges, writability, and enum options. Uses only the control channel
//! (no streaming), so it works even when the GVSP stream is blocked.
//!
//!   cargo run --example gev_features --features gev -- 192.168.0.2

use std::io::Read;
use std::net::{IpAddr, Ipv4Addr, SocketAddr};

use cameleon_genapi::store::{DefaultCacheStore, DefaultNodeStore, DefaultValueStore, NodeId, NodeStore};
use cameleon_genapi::interface::{IBoolean, ICategory, IEnumeration, IFloat, IInteger};
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
    let ip: Ipv4Addr = std::env::args().nth(1).unwrap_or_else(|| "192.168.0.2".into()).parse()?;
    let rt = tokio::runtime::Builder::new_current_thread().enable_all().build()?;
    let mut dev = rt.block_on(GigeDevice::open(SocketAddr::new(IpAddr::V4(ip), gvcp::GVCP_PORT)))?;
    rt.block_on(dev.claim_control())?;

    let (store, mut ctxt) = load_genapi(&rt, &mut dev)?;
    let root = store.id_by_name("Root").ok_or_else(|| anyhow::anyhow!("no Root category"))?;
    let mut b = Bridge { rt: &rt, dev: &mut dev };
    let mut seen = std::collections::HashSet::new();
    walk(root, &store, &mut b, &mut ctxt, 0, &mut seen);

    rt.block_on(b.dev.release_control()).ok();
    Ok(())
}

fn walk(
    nid: NodeId,
    store: &DefaultNodeStore,
    dev: &mut Bridge,
    ctxt: &mut ValueCtxt<DefaultValueStore, DefaultCacheStore>,
    depth: usize,
    seen: &mut std::collections::HashSet<u32>,
) {
    let name = nid.name(store).to_string();
    let pad = "  ".repeat(depth);

    if let Some(cat) = nid.as_icategory_kind(store) {
        println!("{pad}[{name}]");
        for &child in cat.nodes(store) {
            // NodeId is Copy; guard against cycles/dups by raw index.
            let raw = format!("{child:?}");
            if seen.insert(hash(&raw)) {
                walk(child, store, dev, ctxt, depth + 1, seen);
            }
        }
        return;
    }

    if let Some(en) = nid.as_ienumeration_kind(store) {
        let w = en.is_writable(dev, store, ctxt).unwrap_or(false);
        let cur = en.current_entry(dev, store, ctxt).ok()
            .and_then(|e| e.expect_enum_entry(store).ok().map(|x| x.symbolic().to_string()))
            .unwrap_or_default();
        let opts: Vec<String> = en.entries(store).iter()
            .filter_map(|e| e.expect_enum_entry(store).ok().map(|x| x.symbolic().to_string()))
            .collect();
        println!("{pad}{name}: enum = {cur} {}  options=[{}]", rw(w), opts.join(", "));
    } else if let Some(i) = nid.as_iinteger_kind(store) {
        let w = i.is_writable(dev, store, ctxt).unwrap_or(false);
        let v = i.value(dev, store, ctxt).unwrap_or(0);
        let lo = i.min(dev, store, ctxt).unwrap_or(0);
        let hi = i.max(dev, store, ctxt).unwrap_or(0);
        let unit = i.unit(store).unwrap_or("");
        println!("{pad}{name}: int = {v} [{lo}..{hi}] {unit} {}", rw(w));
    } else if let Some(f) = nid.as_ifloat_kind(store) {
        let w = f.is_writable(dev, store, ctxt).unwrap_or(false);
        let v = f.value(dev, store, ctxt).unwrap_or(0.0);
        let lo = f.min(dev, store, ctxt).unwrap_or(0.0);
        let hi = f.max(dev, store, ctxt).unwrap_or(0.0);
        let unit = f.unit(store).unwrap_or("");
        println!("{pad}{name}: float = {v:.3} [{lo:.3}..{hi:.3}] {unit} {}", rw(w));
    } else if let Some(bn) = nid.as_iboolean_kind(store) {
        let w = bn.is_writable(dev, store, ctxt).unwrap_or(false);
        let v = bn.value(dev, store, ctxt).unwrap_or(false);
        println!("{pad}{name}: bool = {v} {}", rw(w));
    } else if nid.as_icommand_kind(store).is_some() {
        println!("{pad}{name}: command");
    } else {
        println!("{pad}{name}: (other)");
    }
}

fn rw(w: bool) -> &'static str { if w { "(writable)" } else { "(read-only)" } }
fn hash(s: &str) -> u32 { s.bytes().fold(2166136261u32, |h, b| (h ^ b as u32).wrapping_mul(16777619)) }

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
