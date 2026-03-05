use embedded_hal::delay::DelayNs;
use esp_hal::gpio::{InputPin, OutputPin};
use esp_hal::peripherals;
use esp_hal::time::{Rate};
use esp_hal::i2c::master as i2c;
use esp_hal::uart;
use esp_hal::rmt;
use bno055::{Bno055, AxisRemap, BNO055AxisSign};

#[derive(Debug)]
pub enum BoardError {
    I2cConfig(i2c::ConfigError),
    UartConfig(uart::ConfigError),
    UartRx(uart::RxError),
    Bno055(bno055::Error<i2c::Error>),
    RmtError(rmt::Error)
}

pub struct NavHatBoard<'d> {
    pub bno055: Bno055<i2c::I2c<'d, esp_hal::Blocking>>,
    uart_rx: uart::UartRx<'d, esp_hal::Blocking>,
}

impl<'d> NavHatBoard<'d> {
    pub fn new<
    SDA: OutputPin + InputPin + 'd,
    SCL: OutputPin + InputPin + 'd,
    TX: OutputPin + 'd,
    RX: InputPin + 'd>(
        i2c0: peripherals::I2C0<'d>,
        uart1: peripherals::UART1<'d>,
        sda: SDA,
        scl: SCL,
        tx: TX,
        rx: RX) -> Result<Self, BoardError> {
        let i2c0 = i2c::I2c::new(
            i2c0,
            i2c::Config::default().with_frequency(Rate::from_khz(400)),
        ).map_err(BoardError::I2cConfig)?
        .with_sda(sda)
        .with_scl(scl);

        let bno055 = Bno055::new(i2c0).with_alternative_address();
        let (uart_rx, _) = uart::Uart::new(
            uart1, 
            uart::Config::default().with_baudrate(9600))
        .map_err(BoardError::UartConfig)?
        .with_tx(tx)
        .with_rx(rx)
        .split();

        return Ok(Self {
            bno055,
            uart_rx,
        });
    }

    pub fn setup_bno055(&mut self, mut delay: &mut dyn DelayNs) -> Result<(), BoardError> {
        self.bno055.init(&mut delay).map_err(BoardError::Bno055)?;

        // self.bno055.set_axis_remap(AxisRemap::builder().swap_x_with(BNO055AxisConfig::AXIS_AS_Y).build().unwrap()).map_err(BoardError::Bno055)?;
        // self.bno055.set_axis_sign(BNO055AxisSign::Y_NEGATIVE).map_err(BoardError::Bno055)?;

        self.bno055.set_axis_remap(AxisRemap::builder().build().unwrap()).map_err(BoardError::Bno055)?;
        self.bno055.set_axis_sign(BNO055AxisSign::empty()).map_err(BoardError::Bno055)?;

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