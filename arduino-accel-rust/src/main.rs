#![feature(asm, lang_items, unwind_attributes, abi_avr_interrupt)]

#![no_std]
#![no_main]

extern crate arduino;
// extern crate itoa;

use arduino::{DDRB, PORTB, serial, timer1, prelude, TCNT1};
use prelude::*;
use core::ptr::write_volatile;
use core::ptr::read_volatile;

static mut TEST: u8 = 'A' as u8;

pub mod write_to {
    use core::cmp::min;
    use core::fmt;

    pub struct WriteTo<'a> {
        buffer: &'a mut [u8],
        // on write error (i.e. not enough space in buffer) this grows beyond
        // `buffer.len()`.
        used: usize,
    }

    impl<'a> WriteTo<'a> {
        pub fn new(buffer: &'a mut [u8]) -> Self {
            WriteTo { buffer, used: 0 }
        }

        pub fn as_str(self) -> Option<&'a str> {
            if self.used <= self.buffer.len() {
                // only successful concats of str - must be a valid str.
                use core::str::from_utf8_unchecked;
                Some(unsafe { from_utf8_unchecked(&self.buffer[..self.used]) })
            } else {
                None
            }
        }
    }

    impl<'a> fmt::Write for WriteTo<'a> {
        fn write_str(&mut self, s: &str) -> fmt::Result {
            if self.used > self.buffer.len() {
                return Err(fmt::Error);
            }
            let remaining_buf = &mut self.buffer[self.used..];
            let raw_s = s.as_bytes();
            let write_num = min(raw_s.len(), remaining_buf.len());
            remaining_buf[..write_num].copy_from_slice(&raw_s[..write_num]);
            self.used += raw_s.len();
            if write_num < raw_s.len() {
                Err(fmt::Error)
            } else {
                Ok(())
            }
        }
    }

    pub fn show<'a>(buffer: &'a mut [u8], args: fmt::Arguments) -> Result<&'a str, fmt::Error> {
        let mut w = WriteTo::new(buffer);
        fmt::write(&mut w, args)?;
        w.as_str().ok_or(fmt::Error)
    }
}

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

    unsafe {
        // asm!("CLI");
        asm!("SEI");
    }

    // for &b in b"p1\n" { serial::transmit(b); }

    loop {
        // Set all pins on PORTB to high.
        unsafe { write_volatile(PORTB, 0xFF) }

        // for &b in b"p2\n" { serial::transmit(b); }

        small_delay();
        // for &b in b"p3\n" { serial::transmit(b); }

        // Set all pins on PORTB to low.
        unsafe { write_volatile(PORTB, 0x00) }
        // for &b in b"p4\n" { serial::transmit(b); }

        small_delay();
        // for &b in b"p5\n" { serial::transmit(b); }

        unsafe { serial::transmit(TEST as u8) }
        // for &b in b"p6\n" { serial::transmit(b); }
        unsafe {

            // for &b in b"p7\n" { serial::transmit(b); }
            let x: u16 = read_volatile(TCNT1);
            // let l = *TCNT1 as u8;
            // let u = (*TCNT1 >> 8) as u8;
            let l = x as u8;
            let u = (x >> 8) as u8;
            serial::transmit(l);
            serial::transmit(u);

            // let mut bytes = [b'\0'; 20];
            // let n = itoa::write(&mut bytes[..], 128u64)?;



            // let mut buf = [0u8; 64];
            // let s: &str = write_to::show(
            //     &mut buf,
            //     format_args!("write some stuff {:?}: {}", "foo", 42),
            // ).unwrap();

            // serial::transmit('x' as u8);
            // for b in s.bytes() {
            //     serial::transmit(b);
            // }
            // serial::transmit('x' as u8);


            let mut chars = [b'\0'; 20];

            // let mut n = *TCNT1;
            let mut n = read_volatile(TCNT1);
            // let mut n: u16 = 12340;
            let mut count = 0;
            while n > 0 {
                let r = n % 10;
                // serial::transmit(r as u8 + 48);
                chars[count] = r as u8 + 48;
                n /= 10;
                count += 1;
            }
            for i in 1..(count+1) {
                serial::transmit(chars[count-i]);
            }


            // for &b in b"p6\n" { serial::transmit(b); }

        }
        for &b in b" OK\n" {
            serial::transmit(b);
        }

    }
}

/// A small busy loop.
fn small_delay() {
    // for &b in b"d1\n" { serial::transmit(b); }

    for _ in 0..100000 {
        // serial::transmit(b'z');
        unsafe { asm!("" :::: "volatile")}
    }
    // for &b in b"d2\n" { serial::transmit(b); }
}

#[no_mangle]
// pub unsafe extern "avr-interrupt" fn _ivr_timer1_compare_a() {
pub unsafe extern "C" fn __vector_11() {

    // let prev_value = read_volatile(PORTB);
    // write_volatile(PORTB, prev_value ^ PINB5);

    for &b in b" --- interrupt\n" {
        serial::transmit(b);
    }

    // TEST = (TEST + (1 as u8)) % ('A' as u8 + 26);
    TEST = ((TEST - ('A' as u8)) + 1) % 26 + 'A' as u8;

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

