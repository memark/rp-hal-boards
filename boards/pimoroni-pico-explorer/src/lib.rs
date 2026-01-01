#![no_std]

pub extern crate rp2040_hal as hal;

#[cfg(feature = "rt")]
extern crate cortex_m_rt;

#[cfg(feature = "rt")]
pub use hal::entry;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
///
/// This currently assumes an rp-pico or pimoroni-pico-lipo is used as the brains.
/// Currently those are the only boards that have the right pin out to be able to be used
#[cfg(feature = "boot2")]
#[link_section = ".boot2"]
#[no_mangle]
#[used]
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

use cortex_m::delay::Delay;
use embedded_graphics::{draw_target::DrawTarget, pixelcolor::Rgb565, prelude::RgbColor};
use embedded_hal::delay::DelayNs;
use embedded_hal_0_2::{
    adc::{Channel, OneShot},
    digital::v2::InputPin,
    spi::MODE_0,
};
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use fugit::RateExtU32;
pub use hal::pac;
use hal::{
    adc::Adc,
    gpio::{
        bank0::*, FunctionNull, FunctionPwm, FunctionSio, FunctionSioInput, FunctionSpi, Pin,
        PullNone, PullUp, SioOutput,
    },
    pac::{RESETS, SPI0},
    sio::SioGpioBank0,
    spi::{Enabled, Spi},
};
use mipidsi::{interface::SpiInterface, models::ST7789, Builder, Display, NoResetPin};
use static_cell::StaticCell;

pub mod all_pins {
    hal::bsp_pins!(
        Gpio0 { name: gpio0 },
        Gpio1 { name: gpio1 },
        Gpio2 { name: gpio2 },
        Gpio3 { name: gpio3 },
        Gpio4 { name: gpio4 },
        Gpio5 { name: gpio5 },
        Gpio6 { name: gpio6 },
        Gpio7 { name: gpio7 },
        Gpio8 {
            name: motor1_neg,
            aliases: { FunctionPwm, PullNone: Motor1Neg }
        },
        Gpio9 {
            name: motor1_pos,
            aliases: { FunctionPwm, PullNone: Motor1Pos }
        },
        Gpio10 {
            name: motor2_neg,
            aliases: { FunctionPwm, PullNone: Motor2Neg }
        },
        Gpio11 {
            name: motor2_pos,
            aliases: { FunctionPwm, PullNone: Motor2Pos }
        },
        Gpio12 { name: switch_a },
        Gpio13 { name: switch_b },
        Gpio14 { name: switch_x },
        Gpio15 { name: switch_y },
        Gpio16 {
            name: spi_miso,
            aliases: { FunctionSpi, PullNone: Miso }
        },
        Gpio17 {
            name: lcd_cs,
            aliases: { FunctionSpi, PullNone: LcdCs }
        },
        Gpio18 {
            name: spi_sclk,
            aliases: { FunctionSpi, PullNone: Sclk }
        },
        Gpio19 {
            name: spi_mosi,
            aliases: { FunctionSpi, PullNone: Mosi }
        },
        Gpio20 {
            name: i2c_sda,
            aliases: { FunctionI2C, PullUp: Sda }
        },
        Gpio21 {
            name: i2c_scl,
            aliases: { FunctionI2C, PullUp: Scl }
        },
        Gpio22 { name: i2c_int },
        Gpio23 { name: b_power_save },
        Gpio24 { name: vbus_detect },
        Gpio25 { name: led },
        Gpio26 { name: adc0 },
        Gpio27 { name: adc1 },
        Gpio28 { name: adc2 },
        Gpio29 {
            name: voltage_monitor
        },
    );
}

// Can't use `hal::bsp_pins!` here because some pins are not set to their reset state
pub struct Pins {
    pub gpio0: Pin<Gpio0, FunctionNull, PullNone>,
    pub gpio1: Pin<Gpio1, FunctionNull, PullNone>,
    pub gpio2: Pin<Gpio2, FunctionNull, PullNone>,
    pub gpio3: Pin<Gpio3, FunctionNull, PullNone>,
    pub gpio4: Pin<Gpio4, FunctionNull, PullNone>,
    pub gpio5: Pin<Gpio5, FunctionNull, PullNone>,
    pub gpio6: Pin<Gpio6, FunctionNull, PullNone>,
    pub gpio7: Pin<Gpio7, FunctionNull, PullNone>,
    pub i2c_sda: all_pins::Sda,
    pub i2c_scl: all_pins::Scl,
    pub i2c_int: Pin<Gpio22, FunctionSioInput, PullUp>,
    pub b_power_save: Pin<Gpio23, FunctionNull, PullNone>,
    pub vbus_detect: Pin<Gpio24, FunctionNull, PullNone>,
    pub led: Pin<Gpio25, FunctionNull, PullNone>,
    pub adc0: Pin<Gpio26, FunctionNull, PullNone>,
    pub adc1: Pin<Gpio27, FunctionNull, PullNone>,
    pub adc2: Pin<Gpio28, FunctionNull, PullNone>,
    pub voltage_monitor: Pin<Gpio29, FunctionNull, PullNone>,
}

pub const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;

pub enum Button {
    A,
    B,
    X,
    Y,
}

pub enum Motor {
    _1,
    _2,
}

pub enum MotorAction {
    Forward(f32),
    Reverse(f32),
    Stop,
}

pub type Screen<'a> = Display<
    SpiInterface<
        'a,
        ExclusiveDevice<
            Spi<
                Enabled,
                SPI0,
                (
                    Pin<Gpio19, FunctionSpi, PullNone>,
                    Pin<Gpio18, FunctionSpi, PullNone>,
                ),
            >,
            Pin<Gpio17, FunctionSio<SioOutput>, PullNone>,
            NoDelay,
        >,
        Pin<Gpio16, FunctionSio<SioOutput>, PullNone>,
    >,
    ST7789,
    NoResetPin,
>;

pub struct PicoExplorer<'a> {
    pub a: Pin<Gpio12, FunctionSioInput, PullUp>,
    pub b: Pin<Gpio13, FunctionSioInput, PullUp>,
    pub x: Pin<Gpio14, FunctionSioInput, PullUp>,
    pub y: Pin<Gpio15, FunctionSioInput, PullUp>,
    adc: Adc,
    pub screen: Screen<'a>,
}

static SPI_BUFFER: StaticCell<[u8; 1024]> = StaticCell::new();

// Newtype wrapper implementing the delay trait from EH 1.0.
struct Hal10Delay<'a>(&'a mut Delay);

// This block is copied from (the currently unreleased) commit 5573370 of the cortex-m crate.
impl<'a> DelayNs for Hal10Delay<'a> {
    #[inline]
    fn delay_ns(&mut self, ns: u32) {
        // from the rp2040-hal:
        let us = ns / 1000 + if ns % 1000 == 0 { 0 } else { 1 };
        // With rustc 1.73, this can be replaced by:
        // let us = ns.div_ceil(1000);
        Delay::delay_us(self.0, us)
    }

    #[inline]
    fn delay_us(&mut self, us: u32) {
        Delay::delay_us(self.0, us)
    }

    #[inline]
    fn delay_ms(&mut self, ms: u32) {
        Delay::delay_ms(self.0, ms)
    }
}

impl<'a> PicoExplorer<'a> {
    pub fn new(
        io: pac::IO_BANK0,
        pads: pac::PADS_BANK0,
        sio: SioGpioBank0,
        spi0: SPI0,
        adc: Adc,
        resets: &mut RESETS,
        delay: &mut Delay,
    ) -> (Self, Pins) {
        let internal_pins = all_pins::Pins::new(io, pads, sio, resets);

        let a = internal_pins.switch_a.into_pull_up_input();
        let b = internal_pins.switch_b.into_pull_up_input();
        let x = internal_pins.switch_x.into_pull_up_input();
        let y = internal_pins.switch_y.into_pull_up_input();

        internal_pins.motor1_pos.into_function::<FunctionPwm>();
        internal_pins.motor1_neg.into_function::<FunctionPwm>();
        internal_pins.motor2_pos.into_function::<FunctionPwm>();
        internal_pins.motor2_neg.into_function::<FunctionPwm>();

        let dc = internal_pins.spi_miso.reconfigure();
        let cs = internal_pins.lcd_cs.reconfigure();
        let spi_sclk = internal_pins.spi_sclk.reconfigure();
        let spi_mosi = internal_pins.spi_mosi.reconfigure();

        let spi_bus =
            Spi::new(spi0, (spi_mosi, spi_sclk)).init(resets, 125u32.MHz(), 16u32.MHz(), MODE_0);
        let spi_device = ExclusiveDevice::new_no_delay(spi_bus, cs).unwrap();
        let spi_buffer = SPI_BUFFER.init([0; _]);
        let di = SpiInterface::new(spi_device, dc, spi_buffer);
        let mut screen = Builder::new(ST7789, di)
            .display_size(240, 240)
            .init(&mut Hal10Delay(delay))
            .unwrap();
        screen.clear(Rgb565::BLACK).unwrap();

        (
            PicoExplorer {
                a,
                b,
                x,
                y,
                adc,
                screen,
            },
            Pins {
                gpio0: internal_pins.gpio0.reconfigure(),
                gpio1: internal_pins.gpio1.reconfigure(),
                gpio2: internal_pins.gpio2.reconfigure(),
                gpio3: internal_pins.gpio3.reconfigure(),
                gpio4: internal_pins.gpio4.reconfigure(),
                gpio5: internal_pins.gpio5.reconfigure(),
                gpio6: internal_pins.gpio6.reconfigure(),
                gpio7: internal_pins.gpio7.reconfigure(),
                i2c_sda: internal_pins.i2c_sda.reconfigure(),
                i2c_scl: internal_pins.i2c_scl.reconfigure(),
                i2c_int: internal_pins.i2c_int.reconfigure(),
                b_power_save: internal_pins.b_power_save.reconfigure(),
                vbus_detect: internal_pins.vbus_detect.reconfigure(),
                led: internal_pins.led.reconfigure(),
                adc0: internal_pins.adc0.reconfigure(),
                adc1: internal_pins.adc1.reconfigure(),
                adc2: internal_pins.adc2.reconfigure(),
                voltage_monitor: internal_pins.voltage_monitor.reconfigure(),
            },
        )
    }

    pub fn is_pressed(&self, button: Button) -> bool {
        use Button::*;
        match button {
            A => self.a.is_low().unwrap(),
            B => self.b.is_low().unwrap(),
            X => self.x.is_low().unwrap(),
            Y => self.y.is_low().unwrap(),
        }
    }

    pub fn get_adc<Pin: Channel<Adc, ID = u8>>(&mut self, channel: &mut Pin) -> f32 {
        // scale raw 12-bit adc value to 0 .. 1 float
        let adc_value: u16 = self.adc.read(channel).unwrap();
        let result: f32 = f32::from(adc_value) / f32::from(1u16 << 12);
        result.clamp(0.0, 1.0)
    }
}
