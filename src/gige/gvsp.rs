//! GVSP (GigE Vision Streaming Protocol) packet parsing and frame reassembly.
//!
//! Wire layouts, offsets and the spec-derived golden-byte tests are adapted from
//! the MIT-licensed `viva-gige` / `viva-genicam` crates (`gvsp.rs`,
//! <https://github.com/VitalyVorobyev/viva-genicam>). This is a copy-minimal,
//! std-only reimplementation owned by the app.

use std::time::Instant;

/// Standard GVSP header size (8 bytes): status(2) block_id(2) format(1) id(3).
const GVSP_HEADER_SIZE: usize = 8;
/// Extended-ID GVSP header (GigE Vision 2.0+): 20 bytes, 64-bit block id and
/// 32-bit packet id after the standard header.
const GVSP_EXTENDED_HEADER_SIZE: usize = 20;
/// Bit 7 of the packet-format byte marks the extended-ID header.
const EXTENDED_ID_FLAG: u8 = 0x80;

/// GVSP leader payload type for image data (`0x4001` is the image + chunk
/// variant real cameras use and must be treated as image too — viva-gige lost
/// it to a truncating `as u8` cast).
const PAYLOAD_TYPE_IMAGE: u16 = 0x0001;
const PAYLOAD_TYPE_IMAGE_EXTENDED_CHUNK: u16 = 0x4001;

/// A parsed GVSP packet. Payload bytes borrow the receive buffer to avoid a copy
/// on the hot path.
#[derive(Debug)]
pub enum GvspPacket<'a> {
    /// Start-of-frame leader carrying geometry.
    Leader {
        block_id: u64,
        width: u32,
        height: u32,
        pixel_format: u32,
    },
    /// Payload data packet (`packet_id` is 1-based on the wire).
    Payload {
        block_id: u64,
        packet_id: u32,
        data: &'a [u8],
    },
    /// End-of-frame trailer.
    Trailer { block_id: u64 },
}

/// Reason a datagram was not a usable GVSP packet.
#[derive(Debug, PartialEq, Eq)]
pub enum GvspError {
    Truncated,
    UnsupportedFormat,
    UnsupportedPayloadType,
}

/// Parse one UDP datagram into a GVSP packet.
///
/// Header layout (8 bytes): `status(2) | block_id(2) | packet_format(1) |
/// packet_id(3)`; the extended-ID header adds a 64-bit block id at offset 8 and
/// a 32-bit packet id at offset 16. The packet format is the low nibble of
/// byte 4; `0x01` leader, `0x02` trailer, `0x03` payload.
pub fn parse_packet(payload: &[u8]) -> Result<GvspPacket<'_>, GvspError> {
    if payload.len() < GVSP_HEADER_SIZE {
        return Err(GvspError::Truncated);
    }
    let format_byte = payload[4];
    let extended = (format_byte & EXTENDED_ID_FLAG) != 0;
    let format = format_byte & 0x0F;

    let (block_id, packet_id, offset) = if extended {
        if payload.len() < GVSP_EXTENDED_HEADER_SIZE {
            return Err(GvspError::Truncated);
        }
        let block_id = u64::from_be_bytes([
            payload[8], payload[9], payload[10], payload[11], payload[12], payload[13],
            payload[14], payload[15],
        ]);
        let packet_id = u32::from_be_bytes([payload[16], payload[17], payload[18], payload[19]]);
        (block_id, packet_id, GVSP_EXTENDED_HEADER_SIZE)
    } else {
        let block_id = u16::from_be_bytes([payload[2], payload[3]]) as u64;
        let packet_id = u32::from_be_bytes([0, payload[5], payload[6], payload[7]]);
        (block_id, packet_id, GVSP_HEADER_SIZE)
    };

    match format {
        0x01 => parse_leader(block_id, &payload[offset..]),
        0x03 => Ok(GvspPacket::Payload {
            block_id,
            packet_id,
            data: &payload[offset..],
        }),
        0x02 => Ok(GvspPacket::Trailer { block_id }),
        _ => Err(GvspError::UnsupportedFormat),
    }
}

/// Leader payload: `reserved(2) | payload_type(2) | timestamp(8) |
/// pixel_format(4) | size_x(4) | size_y(4)` then offsets/padding (ignored).
fn parse_leader(block_id: u64, payload: &[u8]) -> Result<GvspPacket<'_>, GvspError> {
    if payload.len() < 24 {
        return Err(GvspError::Truncated);
    }
    let payload_type = u16::from_be_bytes([payload[2], payload[3]]);
    if payload_type != PAYLOAD_TYPE_IMAGE && payload_type != PAYLOAD_TYPE_IMAGE_EXTENDED_CHUNK {
        return Err(GvspError::UnsupportedPayloadType);
    }
    let pixel_format = u32::from_be_bytes([payload[12], payload[13], payload[14], payload[15]]);
    let width = u32::from_be_bytes([payload[16], payload[17], payload[18], payload[19]]);
    let height = u32::from_be_bytes([payload[20], payload[21], payload[22], payload[23]]);
    Ok(GvspPacket::Leader {
        block_id,
        width,
        height,
        pixel_format,
    })
}

/// Grow-only bitmap tracking which packets of a block have arrived.
struct PacketBitmap {
    words: Vec<u64>,
    received: usize,
    total: usize,
}

impl PacketBitmap {
    fn new(total: usize) -> Self {
        Self {
            words: vec![0u64; total.div_ceil(64)],
            received: 0,
            total,
        }
    }

    /// Mark packet `id` received; returns false if it is out of range or a dup.
    fn set(&mut self, id: usize) -> bool {
        if id >= self.total {
            return false;
        }
        let (word, bit) = (id / 64, id % 64);
        let mask = 1u64 << bit;
        if self.words[word] & mask == 0 {
            self.words[word] |= mask;
            self.received += 1;
            true
        } else {
            false
        }
    }

    fn is_complete(&self) -> bool {
        self.received == self.total
    }
}

/// A partially received frame. Payload packets are placed at `id * stride`;
/// [`finish`](Self::finish) returns the compacted image bytes once every
/// expected packet is present.
pub struct FrameAssembly {
    block_id: u64,
    expected: usize,
    stride: usize,
    bitmap: PacketBitmap,
    buffer: Vec<u8>,
    lengths: Vec<usize>,
    /// When the block should be abandoned if still incomplete. Consulted by
    /// [`is_expired`](Self::is_expired); the app currently abandons a stale block
    /// by replacing it on the next leader instead.
    #[allow(dead_code)]
    deadline: Instant,
}

impl FrameAssembly {
    /// Begin reassembling `block_id`, expecting `expected` packets of up to
    /// `stride` image bytes each, abandoning the block at `deadline`.
    pub fn new(block_id: u64, expected: usize, stride: usize, deadline: Instant) -> Self {
        let stride = stride.max(1);
        let expected = expected.max(1);
        Self {
            block_id,
            expected,
            stride,
            bitmap: PacketBitmap::new(expected),
            buffer: vec![0u8; expected * stride],
            lengths: vec![0usize; expected],
            deadline,
        }
    }

    pub fn block_id(&self) -> u64 {
        self.block_id
    }

    #[allow(dead_code)]
    pub fn is_expired(&self, now: Instant) -> bool {
        now >= self.deadline
    }

    /// Place packet `id` (0-based) into the buffer. Ignores out-of-range ids and
    /// packets larger than the stride.
    pub fn ingest(&mut self, id: usize, data: &[u8]) -> bool {
        if id >= self.expected || data.len() > self.stride {
            return false;
        }
        if !self.bitmap.set(id) {
            return true; // duplicate
        }
        self.lengths[id] = data.len();
        let off = id * self.stride;
        self.buffer[off..off + data.len()].copy_from_slice(data);
        true
    }

    /// Return the compacted image bytes when the frame is complete.
    pub fn finish(self) -> Option<Vec<u8>> {
        if !self.bitmap.is_complete() {
            return None;
        }
        // Fast path: every packet before the last is stride-sized, so the buffer
        // is already contiguous — just trim the tail.
        let full_prefix = self
            .lengths
            .iter()
            .take(self.expected - 1)
            .all(|&len| len == self.stride);
        let mut buffer = self.buffer;
        if full_prefix {
            let used = self.stride * (self.expected - 1) + self.lengths[self.expected - 1];
            buffer.truncate(used);
            return Some(buffer);
        }
        // Slow path: a short packet before the last leaves a gap; compact.
        let total: usize = self.lengths.iter().sum();
        let mut out = Vec::with_capacity(total);
        for (i, &len) in self.lengths.iter().enumerate() {
            if len == 0 {
                continue;
            }
            let start = i * self.stride;
            out.extend_from_slice(&buffer[start..start + len]);
        }
        Some(out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    const MONO8: u32 = 0x0108_0001;

    /// A standard GVSP header, assembled from the spec field table.
    fn header(format: u8, block: u16, packet_id: u32) -> Vec<u8> {
        let mut v = vec![0u8, 0]; // status
        v.extend_from_slice(&block.to_be_bytes());
        v.push(format);
        v.extend_from_slice(&packet_id.to_be_bytes()[1..]); // 24-bit packet id
        v
    }

    fn leader(block: u16, w: u32, h: u32, pf: u32, payload_type: u16) -> Vec<u8> {
        let mut v = header(0x01, block, 0);
        v.extend_from_slice(&[0, 0]); // reserved
        v.extend_from_slice(&payload_type.to_be_bytes());
        v.extend_from_slice(&0u64.to_be_bytes()); // timestamp
        v.extend_from_slice(&pf.to_be_bytes());
        v.extend_from_slice(&w.to_be_bytes());
        v.extend_from_slice(&h.to_be_bytes());
        v.extend_from_slice(&[0u8; 8]); // offset_x/y (ignored)
        v
    }

    #[test]
    fn parses_a_spec_leader() {
        let pkt = leader(7, 640, 480, MONO8, PAYLOAD_TYPE_IMAGE);
        match parse_packet(&pkt).expect("leader") {
            GvspPacket::Leader {
                block_id,
                width,
                height,
                pixel_format,
            } => {
                assert_eq!(block_id, 7);
                assert_eq!(width, 640);
                assert_eq!(height, 480);
                assert_eq!(pixel_format, MONO8);
            }
            _ => panic!("expected leader"),
        }
    }

    #[test]
    fn image_extended_chunk_leader_is_still_an_image() {
        // 0x4001 opens a frame just like 0x0001; viva-gige lost this to `as u8`.
        let pkt = leader(1, 8, 8, MONO8, PAYLOAD_TYPE_IMAGE_EXTENDED_CHUNK);
        assert!(matches!(parse_packet(&pkt), Ok(GvspPacket::Leader { .. })));
    }

    #[test]
    fn parses_payload_and_trailer_ids() {
        let mut p = header(0x03, 7, 5);
        p.extend_from_slice(&[1, 2, 3, 4]);
        match parse_packet(&p).expect("payload") {
            GvspPacket::Payload {
                block_id,
                packet_id,
                data,
            } => {
                assert_eq!(block_id, 7);
                assert_eq!(packet_id, 5);
                assert_eq!(data, &[1, 2, 3, 4]);
            }
            _ => panic!("expected payload"),
        }
        let t = header(0x02, 7, 9);
        assert!(matches!(
            parse_packet(&t),
            Ok(GvspPacket::Trailer { block_id: 7 })
        ));
    }

    #[test]
    fn parses_an_extended_id_header() {
        // 20-byte header: 64-bit block id at 8, 32-bit packet id at 16.
        let mut p = vec![0u8, 0]; // status
        p.extend_from_slice(&0u16.to_be_bytes()); // legacy block id
        p.push(0x03 | EXTENDED_ID_FLAG);
        p.extend_from_slice(&[0, 0, 0]); // legacy packet id
        p.extend_from_slice(&0x0000_0001_0000_0009u64.to_be_bytes()); // block id
        p.extend_from_slice(&0x0000_002Au32.to_be_bytes()); // packet id
        p.extend_from_slice(&[9, 9]);
        match parse_packet(&p).expect("extended payload") {
            GvspPacket::Payload {
                block_id,
                packet_id,
                data,
            } => {
                assert_eq!(block_id, 0x0000_0001_0000_0009);
                assert_eq!(packet_id, 0x2A);
                assert_eq!(data, &[9, 9]);
            }
            _ => panic!("expected payload"),
        }
    }

    #[test]
    fn truncated_and_unknown_formats_are_errors() {
        assert!(matches!(parse_packet(&[0u8; 4]), Err(GvspError::Truncated)));
        let mut bad = header(0x07, 1, 1); // format 7 is undefined
        bad.extend_from_slice(&[0; 4]);
        assert!(matches!(parse_packet(&bad), Err(GvspError::UnsupportedFormat)));
    }

    #[test]
    fn reassembles_a_full_frame() {
        let stride = 4;
        let image: Vec<u8> = (0..30u8).collect();
        let expected = image.len().div_ceil(stride);
        let mut a = FrameAssembly::new(1, expected, stride, Instant::now() + Duration::from_secs(1));
        for (i, chunk) in image.chunks(stride).enumerate() {
            assert!(a.ingest(i, chunk));
        }
        assert_eq!(a.finish().as_deref(), Some(&image[..]));
    }

    #[test]
    fn incomplete_frame_yields_none() {
        let mut a = FrameAssembly::new(1, 3, 4, Instant::now() + Duration::from_secs(1));
        a.ingest(0, &[1, 2, 3, 4]);
        a.ingest(2, &[9, 10]);
        assert!(a.finish().is_none(), "packet 1 missing");
    }

    #[test]
    fn duplicate_packets_do_not_double_count() {
        let mut a = FrameAssembly::new(1, 2, 4, Instant::now() + Duration::from_secs(1));
        a.ingest(0, &[1, 2, 3, 4]);
        a.ingest(0, &[1, 2, 3, 4]); // dup
        assert!(a.finish().is_none(), "only one of two packets present");
    }
}
