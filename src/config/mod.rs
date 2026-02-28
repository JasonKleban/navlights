mod codec;
mod storage;

pub use storage::FlashStorage;

use musli::{Encode, Decode};
use bno055::BNO055Calibration;

#[derive(Default, Debug, PartialEq, Encode, Decode)]
pub struct Config {
    #[musli(with = codec::calibration_option_adapter)]
    pub bno_calibration_profile: Option<BNO055Calibration>,
    pub true_offset_deg: Option<f32>,
}

impl Config {
    pub fn load() -> Result<Option<Self>, storage::FlashError> {
        FlashStorage::load()
    }

    pub fn store(&self) -> Result<(), storage::FlashError> {
        FlashStorage::store(self)
    }
}