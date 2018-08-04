#![feature(asm, lang_items, unwind_attributes, abi_avr_interrupt)]

#![no_std]
#![no_main]

extern crate arduino;

use arduino::{DDRB, PORTB, serial, timer1, prelude, TCNT1};
use prelude::*;
use core::ptr::write_volatile;
use core::ptr::read_volatile;

static mut STATIC_TEST: u8 = 'A' as u8;

#[no_mangle]
pub extern fn main() {
    // Set all PORTB pins up as outputs
    unsafe { write_volatile(DDRB, 0xFF) }

    const CPU_FREQUENCY_HZ: u64 = 16_000_000;
    const BAUD: u64 = 9600;
    const UBRR: u16 = (CPU_FREQUENCY_HZ / 16 / BAUD - 1) as u16;

    serial::Serial::new(UBRR)
        .character_size(serial::CharacterSize::EightBits)
        .mode(serial::Mode::Asynchronous)
        .parity(serial::Parity::Disabled)
        .stop_bits(serial::StopBits::OneBit)
        .configure();

    for &b in b"start\n" {
        serial::transmit(b);
    }

    without_interrupts(|| {

        const CPU_FREQUENCY_HZ: u64 = 16_000_000;
        const DESIRED_HZ_TIM1: f64 = 2.0;
        const TIM1_PRESCALER: u64 = 1024;
        const INTERRUPT_EVERY_1_HZ_1024_PRESCALER: u16 =
            ((CPU_FREQUENCY_HZ as f64 / (DESIRED_HZ_TIM1 * TIM1_PRESCALER as f64)) as u64 - 1) as u16;

        timer1::Timer::new()
            .waveform_generation_mode(timer1::WaveformGenerationMode::ClearOnTimerMatchOutputCompare)
            .clock_source(timer1::ClockSource::Prescale1024)
            .output_compare_1(Some(INTERRUPT_EVERY_1_HZ_1024_PRESCALER))
            .configure();

    });

    loop {
        // Set all pins on PORTB to high.
        unsafe { write_volatile(PORTB, 0xFF) }

        small_delay();

        // Set all pins on PORTB to low.
        unsafe { write_volatile(PORTB, 0x00) }

        small_delay();

        unsafe {
            serial::transmit(STATIC_TEST as u8);
            serial::transmit(' ' as u8);

            for &b in b"Counter value: " { serial::transmit(b); }

            let mut n = read_volatile(TCNT1);
            let mut buf = [b'\0'; 20];
            let mut count = 0;
            while n > 0 {
                let r = n % 10;
                buf[count] = r as u8 + 48;
                n /= 10;
                count += 1;
            }
            for i in 1..(count+1) {
                serial::transmit(buf[count-i]);
            }

        }
        for &b in b" OK\n" {
            serial::transmit(b);
        }

    }
}

/// A small busy loop.
fn small_delay() {

    for _ in 0..100000 {
        unsafe { asm!("" :::: "volatile")}
    }

}

#[no_mangle]
// pub unsafe extern "avr-interrupt" fn _ivr_timer1_compare_a() {
pub unsafe extern "C" fn __vector_11() {

    for &b in b" --- interrupt\n" {
        serial::transmit(b);
    }

    // Cycle STATIC_TEST through A-Z
    STATIC_TEST = ((STATIC_TEST - ('A' as u8)) + 1) % 26 + 'A' as u8;

    // Reset counter to zero
    write_volatile(TCNT1, 0);

    // Re-enable interrupts
    asm!("SEI");
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
