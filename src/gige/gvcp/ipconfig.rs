//! FORCEIP and the persistent-IP bootstrap registers: giving a GigE Vision
//! camera an address on this host's subnet, temporarily (FORCEIP, until the
//! next power cycle) or permanently (`GevPersistentIPAddress` and friends
//! plus the persistent-IP bit of `GevCurrentIPConfiguration`).
//!
//! Driven by `examples/gev_force_ip.rs`; the viewer itself does not use it.

use std::net::Ipv4Addr;
use std::time::{Duration, Instant};

use super::{
    broadcast_send, collect_replies, decode_ack, encode_command, Device, GvcpError, Status,
    FLAG_ACK_REQUIRED, FLAG_BROADCAST,
};

// ── Opcodes ─────────────────────────────────────────────────────────────────
/// FORCEIP: give the device with a given MAC a temporary IP configuration.
/// Broadcast, since the device may currently hold an address this host
/// cannot route to. 56-byte payload — see [`encode_forceip_payload`].
const FORCEIP_CMD: u16 = 0x0004;
const FORCEIP_ACK: u16 = 0x0005;

// ── Bootstrap registers ───────────────────────────────────────────────────--
/// `GevCurrentIPConfiguration`: bit 0 LLA, bit 1 persistent IP, bit 2 DHCP.
pub const IP_CONFIG_REGISTER: u32 = 0x0014;
/// `GevCurrentIPConfiguration` bit: link-local addressing enabled.
pub const IP_CONFIG_LLA: u32 = 0x1;
/// `GevCurrentIPConfiguration` bit: boot with the persistent IP.
pub const IP_CONFIG_PERSISTENT: u32 = 0x2;
/// `GevCurrentIPConfiguration` bit: DHCP enabled.
pub const IP_CONFIG_DHCP: u32 = 0x4;
/// Persistent IP address / subnet mask / default gateway registers (the
/// address sits in the last word of each 16-byte block).
const PERSISTENT_IP_REGISTER: u32 = 0x064c;
const PERSISTENT_SUBNET_REGISTER: u32 = 0x065c;
const PERSISTENT_GATEWAY_REGISTER: u32 = 0x066c;

impl Device {
    /// Read the persistent (boot-time) IP, subnet mask and gateway.
    pub fn read_persistent_ip(&mut self) -> Result<(Ipv4Addr, Ipv4Addr, Ipv4Addr), GvcpError> {
        Ok((
            Ipv4Addr::from(self.read_register(PERSISTENT_IP_REGISTER)?),
            Ipv4Addr::from(self.read_register(PERSISTENT_SUBNET_REGISTER)?),
            Ipv4Addr::from(self.read_register(PERSISTENT_GATEWAY_REGISTER)?),
        ))
    }

    /// Write the persistent IP, subnet mask and gateway. Takes effect at the
    /// next boot, and only if [`Self::enable_persistent_ip`] has set the
    /// configuration bit. Requires control privilege.
    pub fn write_persistent_ip(&mut self, ip: Ipv4Addr, subnet: Ipv4Addr, gateway: Ipv4Addr) -> Result<(), GvcpError> {
        self.write_register(PERSISTENT_IP_REGISTER, u32::from(ip))?;
        self.write_register(PERSISTENT_SUBNET_REGISTER, u32::from(subnet))?;
        self.write_register(PERSISTENT_GATEWAY_REGISTER, u32::from(gateway))
    }

    /// Read `GevCurrentIPConfiguration` (see the `IP_CONFIG_*` bits).
    pub fn ip_config(&mut self) -> Result<u32, GvcpError> {
        self.read_register(IP_CONFIG_REGISTER)
    }

    /// Write `GevCurrentIPConfiguration`. Requires control privilege.
    pub fn set_ip_config(&mut self, bits: u32) -> Result<(), GvcpError> {
        self.write_register(IP_CONFIG_REGISTER, bits)
    }

    /// Set the persistent-IP bit in `GevCurrentIPConfiguration`, leaving the
    /// others as they are.
    pub fn enable_persistent_ip(&mut self) -> Result<(), GvcpError> {
        let cfg = self.ip_config()?;
        self.set_ip_config(cfg | IP_CONFIG_PERSISTENT)
    }
}

/// The 56-byte FORCEIP payload: reserved(2) | MAC(6) | reserved(12) |
/// IP(4) | reserved(12) | subnet mask(4) | reserved(12) | gateway(4).
pub fn encode_forceip_payload(mac: [u8; 6], ip: Ipv4Addr, subnet: Ipv4Addr, gateway: Ipv4Addr) -> [u8; 56] {
    let mut p = [0u8; 56];
    p[2..8].copy_from_slice(&mac);
    p[20..24].copy_from_slice(&ip.octets());
    p[36..40].copy_from_slice(&subnet.octets());
    p[52..56].copy_from_slice(&gateway.octets());
    p
}

/// Broadcast FORCEIP: tell the device with `mac` to adopt `ip`/`subnet`/
/// `gateway` until its next power cycle. Waits up to `timeout` for the
/// acknowledgement: `Ok(true)` acknowledged, `Ok(false)` no reply (many
/// cameras apply it silently, or answer from an address this host can't
/// hear), `Err` on a send failure or a rejecting status. To make the address
/// stick, open the device at its new address and use
/// [`Device::write_persistent_ip`] + [`Device::enable_persistent_ip`].
pub fn force_ip(
    mac: [u8; 6],
    ip: Ipv4Addr,
    subnet: Ipv4Addr,
    gateway: Ipv4Addr,
    timeout: Duration,
) -> Result<bool, GvcpError> {
    let request_id: u16 = 0x0200;
    let payload = encode_forceip_payload(mac, ip, subnet, gateway);
    let packet = encode_command(FLAG_ACK_REQUIRED | FLAG_BROADCAST, FORCEIP_CMD, request_id, &payload);
    let sockets = broadcast_send(&packet);
    if sockets.is_empty() {
        return Err(GvcpError::Protocol("FORCEIP could not be sent on any interface".into()));
    }
    let mut result: Result<bool, GvcpError> = Ok(false);
    collect_replies(&sockets, Instant::now() + timeout, |buf| {
        let Ok(ack) = decode_ack(buf) else { return false };
        if ack.command != FORCEIP_ACK || ack.request_id != request_id {
            return false; // discovery replies and other chatter share the port
        }
        result = match ack.status {
            Status::Success => Ok(true),
            other => Err(GvcpError::Status(other)),
        };
        true
    });
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn forceip_payload_sits_at_the_specified_offsets() {
        let p = encode_forceip_payload(
            [0xde, 0xad, 0xbe, 0xef, 0xca, 0xfe],
            Ipv4Addr::new(192, 168, 0, 10),
            Ipv4Addr::new(255, 255, 255, 0),
            Ipv4Addr::new(192, 168, 0, 1),
        );
        assert_eq!(p.len(), 56);
        assert_eq!(&p[0..2], &[0, 0]);
        assert_eq!(&p[2..8], &[0xde, 0xad, 0xbe, 0xef, 0xca, 0xfe]);
        assert!(p[8..20].iter().all(|&b| b == 0));
        assert_eq!(&p[20..24], &[192, 168, 0, 10]);
        assert!(p[24..36].iter().all(|&b| b == 0));
        assert_eq!(&p[36..40], &[255, 255, 255, 0]);
        assert!(p[40..52].iter().all(|&b| b == 0));
        assert_eq!(&p[52..56], &[192, 168, 0, 1]);
        // And the command header around it.
        let pkt = encode_command(FLAG_ACK_REQUIRED | FLAG_BROADCAST, FORCEIP_CMD, 0x0200, &p);
        assert_eq!(&pkt[..8], &[0x42, 0x11, 0x00, 0x04, 0x00, 56, 0x02, 0x00]);
    }
}
