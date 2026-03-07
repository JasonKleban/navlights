use esp_hal::{
    Blocking, 
    gpio::{Level, OutputPin}, 
    rmt::{self, Channel, PulseCode, Tx, TxChannelConfig, TxChannelCreator},
};
/// Invariants:
/// - Blocking only
/// - Does not rely on interrupts
/// - Does not allocate
/// - Safe to construct after Peripherals::steal()
pub struct Ws2812<'d> {
    channel: Option<Channel<'d, Blocking, Tx>>,
}

impl<'d> Ws2812<'d> {
    pub fn new<C : TxChannelCreator<'d, Blocking>>(
        rmt_channel : C,
        pin: impl OutputPin + 'd,
    ) -> Result<Self, rmt::Error> {
        let tx_config = TxChannelConfig::default().with_clk_divider(1);

        let channel = Some(rmt_channel.configure_tx(pin, tx_config)?);

        Ok(Self { channel })
    }

    pub fn write(&mut self, pixels: &[[u8; 3]]) {
        let channel = self.channel.take().unwrap();

        let mut symbols: [PulseCode; 128*24 + 2] = [PulseCode::default(); 128*24 + 2];

        let mut idx = 0;

        for pixel in pixels {
            let bytes = [pixel[1], pixel[0], pixel[2]];

            for byte in bytes {
                for bit in (0..8).rev() {
                    let is_one = (byte >> bit) & 1 != 0;

                    symbols[idx] = if is_one {
                        PulseCode::new(Level::High, 64, Level::Low, 36)
                    } else {
                        PulseCode::new(Level::High, 32, Level::Low, 68)
                    };

                    idx += 1;
                }
            }
        }

        // Reset pulse (>50µs low)
        symbols[idx] = PulseCode::new(Level::Low, 4000, Level::Low, 0);
        idx += 1;

        symbols[idx] = PulseCode::end_marker();

        let tx = channel.transmit(&symbols[..=idx]).unwrap();
        self.channel = Some(tx.wait().unwrap());
    }
}