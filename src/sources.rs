//! Source identity, descriptor parsing, and the backend discovery registry.
//!
//! Everything about *selecting* a source lives here: the [`CameraSource`]
//! identity type, the `scheme:` descriptor grammar, and one [`Backend`] entry
//! per compiled-in camera backend. The Source menu, the CLI parser, and
//! refresh all iterate [`backends()`], so adding a backend means adding its
//! module plus one registry entry (and a `KNOWN_SCHEMES` line) in this file.
//!
//! The *running* side — capture threads, per-backend control panels — stays
//! per-backend in `main.rs`; only selection is unified.

/// Identity of an input source. Every variant has a stable string form (the
/// *descriptor*, e.g. `toupcam:<id>`, `file:<path>`) used as the CLI argument
/// format, the persistence format, and the unified picker's key — one scheme
/// per backend.
#[derive(Clone, PartialEq, Eq, Debug)]
pub enum CameraSource {
    None,
    FitsFile(std::path::PathBuf),
    #[cfg(feature = "svbony")]
    SVBony(i32),
    #[cfg(feature = "gev")]
    Gev(String),
    /// ToupTek camera, identified by its opaque enumeration id.
    #[cfg(feature = "toupcam")]
    Toupcam(String),
    /// INDI/INDIGO server address ("host" or "host:port").
    #[cfg(feature = "indi")]
    Indi(String),
}

impl CameraSource {
    /// Descriptor scheme of this source ("file", "svb", …); `None` for
    /// `Self::None`. The key for looking up this source's [`Backend`].
    #[cfg_attr(
        not(any(feature = "svbony", feature = "gev", feature = "toupcam")),
        allow(dead_code)
    )]
    pub fn scheme(&self) -> Option<&'static str> {
        match self {
            CameraSource::None => None,
            CameraSource::FitsFile(_) => Some("file"),
            #[cfg(feature = "svbony")]
            CameraSource::SVBony(_) => Some("svb"),
            #[cfg(feature = "gev")]
            CameraSource::Gev(_) => Some("gev"),
            #[cfg(feature = "toupcam")]
            CameraSource::Toupcam(_) => Some("toupcam"),
            #[cfg(feature = "indi")]
            CameraSource::Indi(_) => Some("indi"),
        }
    }

    /// The stable string form of this source; `None` for `Self::None`.
    pub fn descriptor(&self) -> Option<String> {
        match self {
            CameraSource::None => None,
            CameraSource::FitsFile(p) => Some(format!("file:{}", p.display())),
            #[cfg(feature = "svbony")]
            CameraSource::SVBony(id) => Some(format!("svb:{}", id)),
            #[cfg(feature = "gev")]
            CameraSource::Gev(id) => Some(format!("gev:{}", id)),
            #[cfg(feature = "toupcam")]
            CameraSource::Toupcam(id) => Some(format!("toupcam:{}", id)),
            #[cfg(feature = "indi")]
            CameraSource::Indi(host) => Some(format!("indi:{}", host)),
        }
    }

    /// Parse a descriptor or a bare FITS path. Schemes for backends this
    /// binary was built without are named in the error rather than treated
    /// as unknown.
    pub fn parse_descriptor(s: &str) -> Result<CameraSource, String> {
        let s = s.trim();
        if let Some(path) = s.strip_prefix("file:") {
            return Ok(CameraSource::FitsFile(path.into()));
        }
        if let Some((scheme, rest)) = s.split_once(':') {
            if let Some(b) = backends().iter().find(|b| b.scheme == scheme) {
                return (b.parse)(rest);
            }
            if let Some((_, feature, name)) = KNOWN_SCHEMES.iter().find(|(k, _, _)| *k == scheme) {
                return Err(format!(
                    "this build has no {} support (rebuild with --features {})",
                    name, feature
                ));
            }
        }
        // A bare existing path means a FITS file (drag-onto-app, `open with`).
        if std::path::Path::new(s).exists() {
            return Ok(CameraSource::FitsFile(s.into()));
        }
        let schemes: String = backends().iter().map(|b| format!("{}:/", b.scheme)).collect();
        Err(format!(
            "unrecognized source '{}' (expected a FITS path or {}file:)",
            s, schemes
        ))
    }
}

/// Backend-typed device info for a discovered camera — exactly what that
/// backend's `start_*` function needs to open it. FITS files and INDI servers
/// never appear here; they are addressed, not discovered.
#[derive(Clone)]
pub enum SourceInfo {
    #[cfg(feature = "svbony")]
    SVBony(svbony::CameraInfo),
    #[cfg(feature = "gev")]
    Gev(crate::gev_camera::GevDeviceInfo),
    #[cfg(feature = "toupcam")]
    Toupcam(toupcam::DeviceInfo),
}

/// One discovered, openable device: identity, display strings, and the
/// backend-typed info needed to open it.
pub struct DiscoveredSource {
    pub source: CameraSource,
    /// Device display name ("SVBONY SV305", "Daheng MER2-160-75GM", …).
    pub name: String,
    /// Secondary detail shown in parentheses (serial, IP); may be empty.
    pub detail: String,
    /// Backend display name ("SVBony", "GigE Vision"), the menu label suffix.
    /// Stamped from the registry entry by [`Backend::discover_devices`] so it
    /// can never drift from the group header it is filtered by.
    pub backend: &'static str,
    /// Only read by `start_discovered`, which needs at least one device
    /// backend compiled in to exist at all.
    #[cfg_attr(
        not(any(feature = "svbony", feature = "gev", feature = "toupcam")),
        allow(dead_code)
    )]
    pub info: SourceInfo,
}

impl DiscoveredSource {
    /// `"<name> (<detail>)"` — the row text inside a backend's dialog group.
    pub fn title(&self) -> String {
        if self.detail.is_empty() {
            self.name.clone()
        } else {
            format!("{} ({})", self.name, self.detail)
        }
    }

    /// `"<name> (<detail>) — <backend>"` as shown in the Source menu.
    pub fn label(&self) -> String {
        format!("{} — {}", self.title(), self.backend)
    }
}

/// Inline address-entry row for backends whose devices are addressed rather
/// than (only) discovered — GigE by IP, INDI by host:port.
pub struct ManualEntrySpec {
    /// Field label ("IP", "Server").
    pub label: &'static str,
    /// Placeholder shown in the empty field ("192.168.0.2").
    pub hint: &'static str,
    /// Initial field text at startup (may be empty).
    pub default: &'static str,
    /// Validate the typed address and turn it into a source identity.
    pub make_source: fn(&str) -> Result<CameraSource, String>,
}

/// A selectable backend: its descriptor scheme and how to find its devices.
pub struct Backend {
    /// Descriptor scheme, e.g. `"svb"` in `svb:0`.
    pub scheme: &'static str,
    /// Display name ("SVBony", "GigE Vision") — group header in the Connect
    /// dialog and the suffix on Source menu labels.
    pub name: &'static str,
    /// Parse the part after `scheme:` into a source identity.
    pub parse: fn(&str) -> Result<CameraSource, String>,
    /// Enumerate currently-visible devices; `None` for address-only backends
    /// (INDI), which are connected via a typed address instead.
    pub discover: Option<fn() -> Vec<DiscoveredSource>>,
    /// Address-entry row shown in the Connect dialog, if this backend can be
    /// reached by a typed address.
    pub manual: Option<ManualEntrySpec>,
}

/// Every scheme this codebase knows, compiled in or not, so a descriptor for
/// a missing backend says "rebuild with --features x" instead of "unknown".
const KNOWN_SCHEMES: &[(&str, &str, &str)] = &[
    ("svb", "svbony", "SVBony"),
    ("toupcam", "toupcam", "ToupTek"),
    ("gev", "gev", "GigE"),
    ("indi", "indi", "INDI"),
];

/// Every backend compiled into this binary.
pub fn backends() -> &'static [Backend] {
    static LIST: std::sync::LazyLock<Vec<Backend>> = std::sync::LazyLock::new(|| {
        #[cfg_attr(
            not(any(feature = "svbony", feature = "gev", feature = "toupcam", feature = "indi")),
            allow(unused_mut)
        )]
        let mut v: Vec<Backend> = Vec::new();
        #[cfg(feature = "svbony")]
        v.push(Backend {
            scheme: "svb",
            name: "SVBony",
            parse: parse_svb,
            discover: Some(discover_svbony),
            manual: None,
        });
        #[cfg(feature = "toupcam")]
        v.push(Backend {
            scheme: "toupcam",
            name: "ToupTek",
            parse: parse_toupcam,
            discover: Some(discover_toupcam),
            manual: None,
        });
        #[cfg(feature = "gev")]
        v.push(Backend {
            scheme: "gev",
            name: "GigE Vision",
            parse: parse_gev,
            discover: Some(discover_gev),
            // Reachable-by-unicast cameras don't always answer broadcast
            // discovery, so a raw IP must also work.
            manual: Some(ManualEntrySpec {
                label: "IP",
                hint: "192.168.0.2",
                default: "",
                make_source: manual_gev,
            }),
        });
        #[cfg(feature = "indi")]
        v.push(Backend {
            scheme: "indi",
            name: "INDI",
            parse: parse_indi,
            discover: None,
            manual: Some(ManualEntrySpec {
                label: "Server",
                hint: "localhost:7624",
                default: "localhost",
                make_source: parse_indi,
            }),
        });
        v
    });
    &LIST
}

impl Backend {
    /// Enumerate this backend's currently-visible devices (empty for
    /// address-only backends), each stamped with this entry's group label —
    /// the single owner of `DiscoveredSource::backend`.
    pub fn discover_devices(&self) -> Vec<DiscoveredSource> {
        let Some(discover) = self.discover else { return Vec::new() };
        let mut found = discover();
        for d in &mut found {
            d.backend = self.name;
        }
        found
    }
}

/// Enumerate every discovery-capable backend — startup and the Source menu's
/// Refresh both go through here.
pub fn discover_all() -> Vec<DiscoveredSource> {
    backends().iter().flat_map(Backend::discover_devices).collect()
}

#[cfg(feature = "svbony")]
fn parse_svb(id: &str) -> Result<CameraSource, String> {
    id.parse()
        .map(CameraSource::SVBony)
        .map_err(|_| format!("invalid SVBony camera id '{}'", id))
}

#[cfg(feature = "svbony")]
fn discover_svbony() -> Vec<DiscoveredSource> {
    crate::camera::enumerate()
        .into_iter()
        .map(|c| DiscoveredSource {
            source: CameraSource::SVBony(c.camera_id),
            name: c.name.clone(),
            detail: c.serial.clone(),
            backend: "", // stamped by discover_devices

            info: SourceInfo::SVBony(c),
        })
        .collect()
}

#[cfg(feature = "toupcam")]
fn parse_toupcam(id: &str) -> Result<CameraSource, String> {
    Ok(CameraSource::Toupcam(id.to_string()))
}

#[cfg(feature = "toupcam")]
fn discover_toupcam() -> Vec<DiscoveredSource> {
    crate::toupcam_camera::enumerate()
        .into_iter()
        .map(|d| DiscoveredSource {
            source: CameraSource::Toupcam(d.id.clone()),
            name: d.display_name.clone(),
            detail: String::new(),
            backend: "", // stamped by discover_devices

            info: SourceInfo::Toupcam(d),
        })
        .collect()
}

#[cfg(feature = "gev")]
fn parse_gev(id: &str) -> Result<CameraSource, String> {
    Ok(CameraSource::Gev(id.to_string()))
}

#[cfg(feature = "gev")]
fn manual_gev(s: &str) -> Result<CameraSource, String> {
    let s = s.trim();
    s.parse::<std::net::Ipv4Addr>()
        .map(|ip| CameraSource::Gev(ip.to_string()))
        .map_err(|_| format!("Invalid IP: '{}'", s))
}

#[cfg(feature = "gev")]
fn discover_gev() -> Vec<DiscoveredSource> {
    crate::gev_camera::enumerate()
        .into_iter()
        .map(|g| DiscoveredSource {
            source: CameraSource::Gev(g.id.clone()),
            name: g.display_name(),
            detail: g.ip.to_string(),
            backend: "", // stamped by discover_devices

            info: SourceInfo::Gev(g),
        })
        .collect()
}

#[cfg(feature = "indi")]
fn parse_indi(host: &str) -> Result<CameraSource, String> {
    let host = host.trim_start_matches("//");
    if host.is_empty() {
        return Err("indi: needs a server address (indi:host[:port])".into());
    }
    Ok(CameraSource::Indi(host.to_string()))
}

#[cfg(test)]
mod source_descriptor_tests {
    use super::CameraSource;

    #[test]
    fn file_roundtrip() {
        let src = CameraSource::parse_descriptor("file:/tmp/x.fits").unwrap();
        assert_eq!(src, CameraSource::FitsFile("/tmp/x.fits".into()));
        assert_eq!(src.descriptor().unwrap(), "file:/tmp/x.fits");
    }

    #[test]
    fn bare_existing_path_is_fits() {
        // Any path that exists parses as a FITS source.
        let dir = std::env::temp_dir().join("astroviewer_desc_test.fits");
        std::fs::write(&dir, b"x").unwrap();
        let src = CameraSource::parse_descriptor(dir.to_str().unwrap()).unwrap();
        assert!(matches!(src, CameraSource::FitsFile(_)));
        let _ = std::fs::remove_file(&dir);
    }

    #[test]
    fn none_has_no_descriptor() {
        assert_eq!(CameraSource::None.descriptor(), None);
    }

    #[test]
    fn unknown_schemes_error() {
        assert!(CameraSource::parse_descriptor("bogus:thing").is_err());
        assert!(CameraSource::parse_descriptor("/no/such/path.fits").is_err());
        #[cfg(not(feature = "indi"))]
        assert!(CameraSource::parse_descriptor("indi:localhost").is_err());
    }

    #[test]
    fn disabled_backend_error_names_the_feature() {
        // Whichever backends are compiled out must produce the "rebuild with
        // --features" hint rather than the generic unknown-scheme error.
        for (scheme, feature, _) in super::KNOWN_SCHEMES {
            if super::backends().iter().any(|b| b.scheme == *scheme) {
                continue;
            }
            let err = CameraSource::parse_descriptor(&format!("{}:x", scheme)).unwrap_err();
            assert!(err.contains(feature), "error for {}: '{}'", scheme, err);
        }
    }

    #[cfg(feature = "indi")]
    #[test]
    fn indi_roundtrip() {
        let src = CameraSource::parse_descriptor("indi:astro.local:7624").unwrap();
        assert_eq!(src, CameraSource::Indi("astro.local:7624".into()));
        assert_eq!(src.descriptor().unwrap(), "indi:astro.local:7624");
        // The scheme also accepts the // form.
        assert_eq!(
            CameraSource::parse_descriptor("indi://astro.local").unwrap(),
            CameraSource::Indi("astro.local".into())
        );
        assert!(CameraSource::parse_descriptor("indi:").is_err());
    }

    #[cfg(feature = "toupcam")]
    #[test]
    fn toupcam_roundtrip() {
        let src = CameraSource::parse_descriptor("toupcam:abc-123").unwrap();
        assert_eq!(src, CameraSource::Toupcam("abc-123".into()));
        assert_eq!(src.descriptor().unwrap(), "toupcam:abc-123");
    }

    #[cfg(feature = "svbony")]
    #[test]
    fn svbony_roundtrip() {
        let src = CameraSource::parse_descriptor("svb:3").unwrap();
        assert_eq!(src, CameraSource::SVBony(3));
        assert_eq!(src.descriptor().unwrap(), "svb:3");
        assert!(CameraSource::parse_descriptor("svb:not-a-number").is_err());
    }
}
