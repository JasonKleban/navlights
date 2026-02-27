use esp_hal::{
    gpio::OutputPin,
    rmt::{
        Rmt,
        TxChannel,
        TxChannelConfig,
        Pulse,
        Channel,
    },
    clock::Clocks,
};

/// WS2812 timing constants in nanoseconds
const T0H_NS: u32 = 400;
const T0L_NS: u32 = 850;
const T1H_NS: u32 = 800;
const T1L_NS: u32 = 450;
const RESET_NS: u32 = 50_000; // 50 µs reset latch

/// Convert nanoseconds to RMT ticks
fn ns_to_ticks(ns: u32, rmt_clock_hz: u32) -> u16 {
    ((ns as u64 * rmt_clock_hz as u64) / 1_000_000_000u64) as u16
}

pub struct Ws2812<'a, C: Channel> {
    channel: TxChannel<'a, C>,
    ticks_per_ns: u32,
}

impl<'a, C: Channel> Ws2812<'a, C> {
    pub fn new(
        rmt: &mut Rmt,
        channel: C,
        pin: impl OutputPin,
        clocks: &Clocks,
    ) -> Self {
        // 80 MHz default APB on ESP32-C3
        let rmt_clock_hz = clocks.apb_clock.to_Hz();

        let tx = rmt
            .channel(channel)
            .configure(
                pin,
                TxChannelConfig::default(),
            )
            .unwrap();

        Self {
            channel: tx,
            ticks_per_ns: rmt_clock_hz,
        }
    }

    /// Write RGB pixels (GRB order required by WS2812)
    pub fn write(&mut self, pixels: &[[u8; 3]]) {
        // Each pixel = 24 bits = 24 RMT symbols
        // Add one reset pulse at end
        const MAX_PIXELS: usize = 64; // adjust for your strip
        const SYMBOLS_PER_PIXEL: usize = 24;
        const TOTAL_SYMBOLS: usize =
            MAX_PIXELS * SYMBOLS_PER_PIXEL + 1;

        let mut symbols: [Pulse; TOTAL_SYMBOLS] =
            [Pulse::new(false, 0, false, 0); TOTAL_SYMBOLS];

        let mut idx = 0;

        let t0h = ns_to_ticks(T0H_NS, self.ticks_per_ns);
        let t0l = ns_to_ticks(T0L_NS, self.ticks_per_ns);
        let t1h = ns_to_ticks(T1H_NS, self.ticks_per_ns);
        let t1l = ns_to_ticks(T1L_NS, self.ticks_per_ns);

        for pixel in pixels {
            // WS2812 expects GRB
            let bytes = [pixel[1], pixel[0], pixel[2]];

            for byte in bytes {
                for bit in (0..8).rev() {
                    let is_one = (byte & (1 << bit)) != 0;

                    symbols[idx] = if is_one {
                        Pulse::new(true, t1h, false, t1l)
                    } else {
                        Pulse::new(true, t0h, false, t0l)
                    };

                    idx += 1;
                }
            }
        }

        // Reset pulse (low for 50µs)
        let reset_ticks =
            ns_to_ticks(RESET_NS, self.ticks_per_ns);

        symbols[idx] =
            Pulse::new(false, reset_ticks, false, 0);

        idx += 1;

        self.channel
            .transmit(&symbols[..idx])
            .unwrap();

        while self.channel.is_transmitting() {}
    }
}