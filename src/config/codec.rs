use bno055::{BNO055Calibration, BNO055_CALIB_SIZE};
use musli::{Encode, Decode};

pub mod calibration_option_adapter {
    use super::*;

    pub fn encode<E>(
        value: &Option<BNO055Calibration>,
        encoder: E,
    ) -> Result<(), E::Error>
    where
        E: musli::Encoder,
    {
        let repr: Option<[u8; BNO055_CALIB_SIZE]> =
            value.as_ref().map(|c| {
                let mut buf = [0u8; BNO055_CALIB_SIZE];
                buf.copy_from_slice(c.as_bytes());
                buf
            });

        repr.encode(encoder)
    }

    pub fn decode<'de, D>(
        decoder: D,
    ) -> Result<Option<BNO055Calibration>, D::Error>
    where
        D: musli::Decoder<'de>,
    {
        let repr: Option<[u8; BNO055_CALIB_SIZE]> =
            Option::<[u8; BNO055_CALIB_SIZE]>::decode(decoder)?;

        Ok(repr.map(|bytes| BNO055Calibration::from_buf(&bytes)))
    }
}