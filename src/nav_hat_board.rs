use embedded_hal::delay::DelayNs;
use esp_hal::clock::CpuClock;
use esp_hal::time::{Rate};
use esp_hal::i2c::master as i2c;
use esp_hal::uart;
use esp_hal::rmt;
use bno055::{Bno055, AxisRemap, BNO055AxisConfig, BNO055AxisSign};
use crate::ws2812_rmt;

#[derive(Debug)]
pub enum BoardError {
    I2cConfig(i2c::ConfigError),
    UartConfig(uart::ConfigError),
    UartRx(uart::RxError),
    Bno055(bno055::Error<i2c::Error>),
    RMT(esp_hal::rmt::Error)
}

pub struct NavHatBoard<'d> {
    pub bno055: Bno055<i2c::I2c<'d, esp_hal::Blocking>>,
    uart_rx: uart::UartRx<'d, esp_hal::Blocking>,
    pub neopixels: ws2812_rmt::Ws2812<'d, esp_hal::rmt::Tx>
}

impl<'d> NavHatBoard<'d> {
    pub fn new() -> Result<Self, BoardError> {
        let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());

        let peripherals = esp_hal::init(config);
        let i2c0: i2c::I2c<'_, esp_hal::Blocking> = i2c::I2c::new(
            peripherals.I2C0, 
            i2c::Config::default().with_frequency(Rate::from_khz(400)))
                .map_err(BoardError::I2cConfig)?
                .with_sda(peripherals.GPIO6)
                .with_scl(peripherals.GPIO7);
        let bno055 = Bno055::new(i2c0)
            .with_alternative_address();
        let (uart_rx, _) = uart::Uart::new(peripherals.UART1, uart::Config::default().with_baudrate(9600))
            .map_err(BoardError::UartConfig)?
            .with_tx(peripherals.GPIO21)
            .with_rx(peripherals.GPIO20)
            .split();
        let mut rmt = rmt::Rmt::new(peripherals.RMT, 80.MHz())?;
        let channel = rmt.channel0;

        let neopixels = ws2812_rmt::Ws2812::new(
            &mut rmt, 
            rmt.channel0,
            peripherals.GPIO3,
            &peripherals.clocks
        ).unwrap();

        return Ok(Self {
            bno055,
            uart_rx,
            neopixels
        });
    }

    pub fn setup_bno055(&mut self, mut delay: &mut dyn DelayNs) -> Result<(), BoardError> {
        let remap = AxisRemap::builder().swap_x_with(BNO055AxisConfig::AXIS_AS_Y).build().unwrap();

        self.bno055.init(&mut delay).map_err(BoardError::Bno055)?;
        self.bno055.set_axis_remap(remap).map_err(BoardError::Bno055)?;
        self.bno055.set_axis_sign(BNO055AxisSign::X_NEGATIVE | BNO055AxisSign::Y_NEGATIVE).map_err(BoardError::Bno055)?;
        self.bno055.set_mode(bno055::BNO055OperationMode::NDOF, &mut delay).map_err(BoardError::Bno055)?;

        Ok(())
    }

    pub fn read_uart_byte(&mut self) -> Result<Option<u8>, BoardError> {
        let mut byte = [0u8; 1];
        
        if self.uart_rx.read_ready() && 
            1 == self.uart_rx.read(&mut byte)
                .map_err(BoardError::UartRx)? {
            Ok(Some(byte[0]))
        } else { 
            Ok(None)
        }
    }
}