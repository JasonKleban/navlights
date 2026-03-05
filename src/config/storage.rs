use core::mem::{size_of};

use crc::{Crc, CRC_32_ISO_HDLC};
use musli::{alloc, context};
use musli::storage::Encoding;

use esp_rom_sys::rom::spiflash::{
    esp_rom_spiflash_read, esp_rom_spiflash_write, esp_rom_spiflash_erase_sector,
};

use super::Config;

unsafe extern "C" {
    unsafe static _config_start: u8;
}

pub fn config_address() -> u32 {
    unsafe { &_config_start as *const u8 as u32 }
}

static FLASH_VERSION: u32 = 1;
static COMMITTED: u32 = 0x00000000;
static UNCOMMITTED: u32 = 0xFFFFFFFF;
static DEADBEEF: u32 = 0xDEADBEEF;

const SECTOR_SIZE: usize = 4096;         // ESP32-C3 flash sector
const MAX_PAYLOAD_SIZE: usize = 1024;     // largest expected config

const CRC32: Crc<u32> = Crc::<u32>::new(&CRC_32_ISO_HDLC);
const ENCODING: Encoding = Encoding::new();

/// Header stored at the beginning of flash for versioning & CRC check
#[repr(C, align(4))]
#[derive(Clone, Copy)]
struct FlashHeader {
    committed: u32,
    deadbeef: u32,
    version: u32,
    payload_len: u32,
    crc32: u32,
}

/// Errors possible when reading/writing flash
#[derive(Debug)]
pub enum FlashError {
    Encode,
    // Decode,
    // Read,
    Write,
    WriteCommitment,
    Erase,
    Invalid,
}

/// Flash storage handler
pub struct FlashStorage;

impl FlashStorage {
    /// Encode `Config` and write to flash with version & CRC
    pub fn store(cfg: &Config) -> Result<(), FlashError> {
        // single contiguous buffer: [header | payload]
        let mut sector_buf = [0u8; SECTOR_SIZE];

        // Split buffer for payload encoding
        let payload_area = &mut sector_buf[size_of::<FlashHeader>()..size_of::<FlashHeader>() + MAX_PAYLOAD_SIZE];

        // musli requires an allocator & context even for fixed-size payloads
        let mut scratch = alloc::ArrayBuffer::new();
        let alloc = alloc::Slice::new(&mut scratch);
        let cx = context::new_in(&alloc);

        // Encode payload directly into payload_area
        let payload_len = ENCODING
            .to_slice_with(&cx, payload_area, cfg)
            .map_err(|_| FlashError::Encode)?;

        // Compute CRC of encoded payload
        let mut digest = CRC32.digest();
        digest.update(&payload_area[..payload_len]);
        let crc = digest.finalize();

        // Construct header in-place at start of sector buffer
        let header = FlashHeader {
            committed: UNCOMMITTED,
            deadbeef: DEADBEEF,
            version: FLASH_VERSION,
            payload_len: payload_len as u32,
            crc32: crc
        };
        let header_bytes = unsafe {
            core::slice::from_raw_parts(&header as *const _ as *const u8, size_of::<FlashHeader>())
        };
        sector_buf[..header_bytes.len()].copy_from_slice(header_bytes);

        let total_len = size_of::<FlashHeader>() + payload_len;
        if total_len > SECTOR_SIZE {
            return Err(FlashError::Invalid);
        }

        // Erase sector before writing
        let erase_res = unsafe { esp_rom_spiflash_erase_sector(config_address() / SECTOR_SIZE as u32) };
        if erase_res != 0 {
            return Err(FlashError::Erase);
        }

        // Write contiguous buffer to flash
        let write_res = unsafe { esp_rom_spiflash_write(config_address(), sector_buf.as_ptr() as *const _, (total_len as u32 + 3) & !3) };
        if write_res != 0 {
            return Err(FlashError::Write);
        }

        // Write committment to flash
        let write_res = unsafe { esp_rom_spiflash_write(config_address(), &COMMITTED as *const _, 4u32) };
        if write_res != 0 {
            return Err(FlashError::WriteCommitment);
        }

        Ok(())
    }

    /// Load `Config` from flash, validate version and CRC
    pub fn load() -> Result<Option<Config>, FlashError>  {
        let mut sector_buf = [0u8; SECTOR_SIZE];

        // Read full sector
        let read_res = unsafe { esp_rom_spiflash_read(config_address(), sector_buf.as_mut_ptr() as *mut _, SECTOR_SIZE as u32) };
        if read_res != 0 {
            return Ok(None);
        }

        // Safety: repr(C) header at start of sector
        let header = unsafe { &*(sector_buf.as_ptr() as *const FlashHeader) };

        if header.committed == UNCOMMITTED || header.deadbeef != DEADBEEF || header.version != FLASH_VERSION {
            return Ok(None);
        }

        let payload_len = header.payload_len as usize;
        if payload_len > MAX_PAYLOAD_SIZE {
            return Ok(None);
        }

        let payload_area = &sector_buf[size_of::<FlashHeader>()..size_of::<FlashHeader>() + payload_len];

        // Validate CRC
        let mut digest = CRC32.digest();
        digest.update(payload_area);
        if digest.finalize() != header.crc32 {
            return Ok(None);
        }

        // Decode payload with musli using temporary allocator
        let mut scratch = alloc::ArrayBuffer::new();
        let alloc = alloc::Slice::new(&mut scratch);
        let cx = context::new_in(&alloc);

        Ok(ENCODING.from_slice_with(&cx, payload_area).ok())
    }
}