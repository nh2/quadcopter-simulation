#![feature(asm, lang_items, unwind_attributes, abi_avr_interrupt)]

#![no_std]
#![no_main]

extern crate arduino;

use arduino::*;
use prelude::*;
use core::ptr::*;

static mut STATIC_TEST: u8 = b'A';

pub fn serial_print(s: &str) {
    for b in s.bytes() {
        serial::transmit(b);
    }
}

pub fn serial_println(s: &str) {
    serial_print(s);
    serial::transmit(b'\r');
    serial::transmit(b'\n');
}

pub fn serial_send_u16_decimal(x: u16) {
    // Allocate a buffer of fitting size, fill it (least significant
    // digits first as we have to modulo-divide), and send it
    // in reverse.

    const BUFSIZE: usize = 5; // u16 never needs more than 5 decimal digits
    let mut buf = [b'\0'; BUFSIZE];

    // Compute size
    let mut n = x;
    let mut chars_needed = 0;
    loop {
        let decimal_digit = n % 10;
        buf[chars_needed] = decimal_digit as u8 + 48;
        n /= 10;
        chars_needed += 1;
        if n == 0 {
            break;
        }
    }

    // Send send buffer
    for i in 1..(chars_needed+1) {
        serial::transmit(buf[chars_needed-i]);
    }
}
pub fn serial_send_u32_decimal(x: u32) {
    // Allocate a buffer of fitting size, fill it (least significant
    // digits first as we have to modulo-divide), and send it
    // in reverse.

    const BUFSIZE: usize = 10; // u64 never needs more than 10 decimal digits
    let mut buf = [b'\0'; BUFSIZE];

    // Compute size
    let mut n = x;
    let mut chars_needed = 0;
    loop {
        let decimal_digit = n % 10;
        buf[chars_needed] = decimal_digit as u8 + 48;
        n /= 10;
        chars_needed += 1;
        if n == 0 {
            break;
        }
    }

    // Send send buffer
    for i in 1..(chars_needed+1) {
        serial::transmit(buf[chars_needed-i]);
    }
}

const CPU_FREQUENCY_HZ: u64 = 16_000_000;
const DESIRED_HZ_TIM1: f64 = 250000.0;
const TIM1_PRESCALER: u64 = 64; // 16 MHz / 64 = 250000 Hz = 4 us
// const TIM1_PRESCALER: u64 = 1;
// const INTERRUPT_EVERY_1_HZ_1024_PRESCALER: u16 =
//     ((CPU_FREQUENCY_HZ as f64 / (DESIRED_HZ_TIM1 * TIM1_PRESCALER as f64)) as u64 - 1) as u16;
const INTERRUPT_EVERY_1M_HZ_1_PRESCALER: u16 =
    ((CPU_FREQUENCY_HZ as f64 / (DESIRED_HZ_TIM1 * TIM1_PRESCALER as f64)) as u64) as u16;


#[no_mangle]
pub extern fn main() {
    // Set all PORTB pins up as outputs
    // unsafe { write_volatile(DDRB, 0xFF) }

    const BAUD: u64 = 9600;
    // const BAUD: u64 = 38400;
    const UBRR: u16 = (CPU_FREQUENCY_HZ / 16 / BAUD - 1) as u16;
    // const BAUD: u64 = 115200;
    // const UBRR: u16 = (CPU_FREQUENCY_HZ / 16 / BAUD) as u16;

    serial::Serial::new(UBRR)
        .character_size(serial::CharacterSize::EightBits)
        .mode(serial::Mode::Asynchronous)
        .parity(serial::Parity::Disabled)
        .stop_bits(serial::StopBits::OneBit)
        .configure();

    serial_println("start");

    without_interrupts(|| {

        timer1::Timer::new()
            .waveform_generation_mode(timer1::WaveformGenerationMode::ClearOnTimerMatchOutputCompare)
            // .clock_source(timer1::ClockSource::Prescale1024)
            // .clock_source(timer1::ClockSource::Prescale256)
            .clock_source(timer1::ClockSource::Prescale64)
            // .clock_source(timer1::ClockSource::Prescale8)
            // .clock_source(timer1::ClockSource::Prescale1)
            // .output_compare_1(Some(INTERRUPT_EVERY_1_HZ_1024_PRESCALER))
            .output_compare_1(Some(INTERRUPT_EVERY_1M_HZ_1_PRESCALER))
            .configure();

    });

    loop {

        // Set all pins on PORTB to high.
        unsafe { write_volatile(PORTB, 0xFF) }

        // small_delay();

        // Set all pins on PORTB to low.
        unsafe { write_volatile(PORTB, 0x00) }

        // small_delay();

        unsafe {
            serial::transmit(STATIC_TEST);
            serial::transmit(b' ');

            serial_print("Counter value: ");

            let counter_value = read_volatile(TCNT1);
            serial_send_u16_decimal(counter_value);

            serial_print(" Comparison value: ");
            serial_send_u16_decimal(INTERRUPT_EVERY_1M_HZ_1_PRESCALER);

            serial_print(" UBRR: ");
            serial_send_u16_decimal(UBRR);

            serial_print(" Micros value: ");
            serial_send_u16_decimal(MICROS_COUNTER as u16); // todo make function for printing u64
            serial_print(" Micros value: ");
            serial_send_u32_decimal(MICROS_COUNTER as u32); // todo make function for printing u64

        }
        serial_println(" OK");

    }
}

static mut MICROS_COUNTER: u32 = 0;

pub fn micros() -> u32 {
    unsafe {
        return MICROS_COUNTER;
    }
}

/// A small busy loop.
fn small_delay() {

    for _ in 0..100000 {
        unsafe { asm!("" :::: "volatile")}
    }

}

// Interrupt vector 11 is the "Timer/Counter1 Compare Match A" interrupt
// handler (see Arduino's `iom328p.h` file.)
#[no_mangle]
// See https://github.com/avr-rust/ruduino/issues/94
// pub unsafe extern "avr-interrupt" fn _ivr_timer1_compare_a() {
// pub unsafe extern "C" fn __vector_11() {
pub unsafe extern "avr-interrupt" fn __vector_11() {

    // serial_println(" --- interrupt");

    // Cycle STATIC_TEST through A-Z
    // STATIC_TEST = ((STATIC_TEST - b'A') + 1) % 26 + b'A';

    // This is quite inaccurate; being 6 instead of 4, it compensates a bit
    // for how many cycles this interrupt handler takes, but the calculation
    // is too primitive (assuming 1 cycle per instruction), as when writing
    // this I didn't have an AVR manual.
    MICROS_COUNTER += 6;

    // Reset counter to zero
    write_volatile(TCNT1, 0);
}

// These do not need to be in a module, but we group them here for clarity.
pub mod std {
    #[lang = "eh_personality"]
    #[no_mangle]
    pub unsafe extern "C" fn rust_eh_personality(_state: (), _exception_object: *mut (), _context: *mut ()) -> () {
    }

    #[lang = "panic_fmt"]
    #[unwind]
    pub extern fn rust_begin_panic(_msg: (), _file: &'static str, _line: u32) -> ! {
        loop { }
    }
}

