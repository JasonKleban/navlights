use esp_rom_sys::rom::spiflash::{
    esp_rom_spiflash_read,
    esp_rom_spiflash_write,
    esp_rom_spiflash_erase_sector,
};

use super::flash_format::FLASH_CONFIG_SIZE;

const CONFIG_FLASH_ADDR: u32 = 0x003F0000; // example - see section below
const SECTOR_SIZE: usize = 4096;

#[derive(Debug)]
pub enum FlashError {
    //Read,
    Write,
    Erase,
    InvalidSize,
}

pub struct FlashStorage;

impl FlashStorage {
    pub fn load() -> Option<[u8; FLASH_CONFIG_SIZE]> {
        let mut bytes = [0u8; FLASH_CONFIG_SIZE];

        let res = unsafe {
            esp_rom_spiflash_read(
                CONFIG_FLASH_ADDR,
                bytes.as_mut_ptr() as *mut _,
                bytes.len() as u32,
            )
        };

        if res != 0 {
            return None;
        }

        Some(bytes)
    }

    pub fn store(bytes: [u8; FLASH_CONFIG_SIZE]) -> Result<(), FlashError> {
        if bytes.len() > SECTOR_SIZE {
            return Err(FlashError::InvalidSize);
        }

        // erase sector
        let erase_res = unsafe {
            esp_rom_spiflash_erase_sector(CONFIG_FLASH_ADDR / SECTOR_SIZE as u32)
        };

        if erase_res != 0 {
            return Err(FlashError::Erase);
        }

        // write must be 4-byte aligned
        let write_res = unsafe {
            esp_rom_spiflash_write(
                CONFIG_FLASH_ADDR,
                bytes.as_ptr() as *const _,
                bytes.len() as u32,
            )
        };

        if write_res != 0 {
            return Err(FlashError::Write);
        }

        Ok(())
    }
}