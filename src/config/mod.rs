pub mod flash_format;
mod storage;

use storage::{ FlashStorage };
use flash_format::{serialize, validate};

pub struct Config {
    pub true_offset_deg: f32,
}

pub fn load() -> Option<Config> {
    let bytes = FlashStorage::load().expect("Unable to read from flash storage");

    validate(&bytes)
}

pub fn store(config : &Config) -> () {
    FlashStorage::store(serialize(&config)).expect("Flash storage should have been successful")
}