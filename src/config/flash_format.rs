use super::Config;
use crc::{Crc, CRC_32_ISO_HDLC};

const EXPECTED_VERSION: u32 = 1;
const CRC32: Crc<u32> = Crc::<u32>::new(&CRC_32_ISO_HDLC);

#[repr(C)]
#[derive(Clone, Copy)]
struct FlashConfig {
    version: u32,

    true_offset_deg: f32,
    
    reserved: u32,
    crc32: u32,
}

pub(crate) const FLASH_CONFIG_SIZE: usize = size_of::<FlashConfig>();

impl FlashConfig {
    fn compute_crc(&self) -> u32 {
        let mut digest = CRC32.digest();

        digest.update(&self.version.to_le_bytes());
        digest.update(&self.true_offset_deg.to_le_bytes());
        digest.update(&self.reserved.to_le_bytes());

        digest.finalize()
    }
}

pub(crate) fn serialize(cfg: &Config) -> [u8; FLASH_CONFIG_SIZE]{
    let mut buf: [u8; FLASH_CONFIG_SIZE] = [0u8; FLASH_CONFIG_SIZE];

    let mut raw = FlashConfig {
        version: EXPECTED_VERSION,
        true_offset_deg: cfg.true_offset_deg,
        reserved: 0,
        crc32: 0,
    };

    raw.crc32 = raw.compute_crc();

    buf[0..4].copy_from_slice(&raw.version.to_le_bytes());
    buf[4..8].copy_from_slice(&raw.true_offset_deg.to_le_bytes());
    buf[8..12].copy_from_slice(&raw.reserved.to_le_bytes());
    buf[12..16].copy_from_slice(&raw.crc32.to_le_bytes());

    buf
}

pub(crate) fn validate(bytes: &[u8]) -> Option<Config> {
    if bytes.len() != FLASH_CONFIG_SIZE {
        return None;
    }

    let inner = FlashConfig {
        version: u32::from_le_bytes(bytes[0..4].try_into().ok()?),
        true_offset_deg: f32::from_le_bytes(bytes[4..8].try_into().ok()?),
        reserved: u32::from_le_bytes(bytes[8..12].try_into().ok()?),
        crc32: u32::from_le_bytes(bytes[12..16].try_into().ok()?),
    };

    if inner.version != EXPECTED_VERSION {
        return None;
    }

    if inner.compute_crc() != inner.crc32 {
        return None;
    }

    Some(Config { true_offset_deg: inner.true_offset_deg })
}