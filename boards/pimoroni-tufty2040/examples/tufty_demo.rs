//! # Tufty2040 Demo Example
//!
//! Draws a circle on the LCD screen and then blinks the user LED on the Tufty 2040.
//!
//! See the `Cargo.toml` file for Copyright and licence details.

#![no_std]
#![no_main]

use embedded_hal_bus::spi::ExclusiveDevice;
use fugit::RateExtU32 as _;
use mipidsi::{interface::SpiInterface, models::ST7789, Builder};
use pimoroni_tufty2040 as tufty;

use rp2040_hal::Spi;
use static_cell::StaticCell;
// The macro for our start-up function
use tufty::entry;

// GPIO traits
use embedded_hal::{
    digital::{OutputPin, PinState},
    spi::MODE_0,
};

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use tufty::hal;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use hal::pac;

use hal::clocks::ClockSource;
use hal::gpio::{FunctionPio0, PullNone};
use hal::Clock;
use hal::Timer;

use tufty::DummyPin;

use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::geometry::Point;
use embedded_graphics::pixelcolor::{Rgb565, RgbColor};
use embedded_graphics::primitives::{Circle, Primitive, PrimitiveStyleBuilder};
use embedded_graphics::Drawable;
use embedded_hal::delay::DelayNs;

static SPI_BUFFER: StaticCell<[u8; 1024]> = StaticCell::new();

#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let cp = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        tufty::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = tufty::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure the timer peripheral for our blinky delay
    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let mut delay = cortex_m::delay::Delay::new(cp.SYST, clocks.system_clock.get_freq().to_Hz());

    // Set the LED to be an output
    let mut led_pin = pins.led.into_push_pull_output();

    pins.lcd_backlight
        .into_push_pull_output_in_state(PinState::High);
    pins.lcd_rd.into_push_pull_output_in_state(PinState::High);

    let display_data = {
        use hal::dma::DMAExt;
        use hal::pio::PIOExt;

        let dma = pac.DMA.split(&mut pac.RESETS);
        let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

        let wr = pins.lcd_wr.reconfigure::<FunctionPio0, PullNone>();
        let d0 = pins.lcd_db0.reconfigure::<FunctionPio0, PullNone>();
        pins.lcd_db1.reconfigure::<FunctionPio0, PullNone>();
        pins.lcd_db2.reconfigure::<FunctionPio0, PullNone>();
        pins.lcd_db3.reconfigure::<FunctionPio0, PullNone>();
        pins.lcd_db4.reconfigure::<FunctionPio0, PullNone>();
        pins.lcd_db5.reconfigure::<FunctionPio0, PullNone>();
        pins.lcd_db6.reconfigure::<FunctionPio0, PullNone>();
        pins.lcd_db7.reconfigure::<FunctionPio0, PullNone>();

        tufty::PioDataLines::new(
            &mut pio,
            clocks.system_clock.freq(),
            wr.id(),
            d0.id(),
            sm0,
            dma.ch0,
        )
    };

    let display_interface = tufty::ParallelDisplayInterface::new(
        pins.lcd_cs.into_push_pull_output_in_state(PinState::High),
        pins.lcd_dc.into_push_pull_output_in_state(PinState::High),
        display_data,
    );

    // let mut display = ST7789::new(display_interface, DummyPin, 240, 320);
    // display.init(&mut delay).unwrap();
    // display.clear(Rgb565::BLUE).unwrap();

    //

    let dc = pins.spi_miso.reconfigure();
    let cs = pins.lcd_cs.reconfigure();
    let spi_sclk = pins.spi_sclk.reconfigure();
    let spi_mosi = pins.spi_mosi.reconfigure();

    let spi_bus = Spi::new(pac.SPI0, (spi_mosi, spi_sclk)).init(
        &mut pac.RESETS,
        125u32.MHz(),
        16u32.MHz(),
        MODE_0,
    );
    let spi_device = ExclusiveDevice::new_no_delay(spi_bus, cs).unwrap();
    let spi_buffer = SPI_BUFFER.init([0; _]);
    let di = SpiInterface::new(spi_device, dc, spi_buffer);
    let mut screen = Builder::new(ST7789, di)
        .display_size(240, 240)
        .init(&mut Hal10Delay(delay))
        .unwrap();
    screen.clear(Rgb565::BLACK).unwrap();

    //

    let style = PrimitiveStyleBuilder::default()
        .fill_color(Rgb565::RED)
        .build();
    Circle::new(Point::new(50, 50), 10)
        .into_styled(style)
        .draw(&mut display)
        .unwrap();

    // Blink the LED at 1 Hz
    loop {
        // LED on, and wait for 500ms
        led_pin.set_high().unwrap();
        timer.delay_ms(500);

        // LED off, and wait for 500ms
        led_pin.set_low().unwrap();
        timer.delay_ms(500);
    }
}

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
