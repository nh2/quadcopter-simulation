#![feature(asm, lang_items, unwind_attributes, abi_avr_interrupt)]

#![no_std]
#![no_main]

extern crate arduino;

use arduino::*;
use prelude::*;
use core::ptr::*;

pub struct DisableAndRestoreInterrupts {
    sreg_to_restore: u8,
}

impl DisableAndRestoreInterrupts {
    #[inline]
    pub fn new() -> DisableAndRestoreInterrupts {
        let sreg = unsafe {
            let sreg = read_volatile(SREG);
            asm!("CLI");
            sreg
        };
        DisableAndRestoreInterrupts { sreg_to_restore: sreg }
    }
}

impl Drop for DisableAndRestoreInterrupts {
    #[inline]
    fn drop(&mut self) {
        unsafe { write_volatile(SREG, self.sreg_to_restore) }
    }
}

pub fn without_interrupts_restore<F, T>(f: F) -> T
    where F: FnOnce() -> T
{
    let _disabled = DisableAndRestoreInterrupts::new();
    f()
}


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

const TIMER_0_PRESCALER: timer0::ClockSource = timer0::ClockSource::Prescale64;
const TIMER_0_PRESCALER_NUM: u32 = 64;

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

        timer0::Timer::new()
            .waveform_generation_mode(timer0::WaveformGenerationMode::Normal)
            .clock_source(TIMER_0_PRESCALER)
            .configure();

        // Enable Timer 0 overflow interrupt
        unsafe { write_volatile(TIMSK0, TOIE0); }
    });

    let mut last_micros = micros();

    loop {

        // Set all pins on PORTB to high.
        unsafe { write_volatile(PORTB, 0xFF) }

        // small_delay();

        // Set all pins on PORTB to low.
        unsafe { write_volatile(PORTB, 0x00) }

        // small_delay();

        unsafe {
            // serial::transmit(STATIC_TEST);
            // serial::transmit(b' ');

            // serial_print("Counter value: ");

            // let counter_value = read_volatile(TCNT0);
            // serial_send_u16_decimal(counter_value as u16);

            // serial_print(" Micros value: ");
            // serial_send_u32_decimal(micros());

            let now = micros();
            if now - last_micros >= 1000000 {
                last_micros = now;

                serial_print("Micros value: ");
                serial_send_u32_decimal(micros());
                serial_println("");
            }

        }
        // serial_println(" OK");

    }
}

// We use Counter 1 (8-bit) with the 64-prescaler,
// so it overflows every 256 * 64 = 16384 cycles.
//
// Thus for a 16 Mhz CPU and u32 to store the counter,
// the overflow-counter will overflow after
// (2**32) / (16000000 / 16384) / (3600 * 24) = 50 days.
static mut NUM_OVERFLOWS_16384_CYCLE_COUNTER: u32 = 0;

const CYCLES_PER_US: u32 = (CPU_FREQUENCY_HZ / 1_000_000) as u32;


// Returns the number of microseconds since the initialisation of
// Counter 0.
// Assumes this setup:
//
//    const TIMER_0_PRESCALER: timer0::ClockSource = timer0::ClockSource::Prescale64;
//    const TIMER_0_PRESCALER_NUM: u32 = 64;
//    timer0::Timer::new()
//        .waveform_generation_mode(timer0::WaveformGenerationMode::Normal)
//        .clock_source(TIMER_0_PRESCALER)
//        .configure();
//
// Overflows after approximately 70 minutes.
// This number will overflow, after approximately 70 minutes.
// On 16 MHz CPUs, this function has a resolution of 4 microseconds
// (i.e. the value returned is always a multiple of 4).
//
// Don't inline or mangle so that we can count `own_cycles` easily.
#[no_mangle]
#[inline(never)]
pub fn micros() -> u32 {
    unsafe {
        // Disable interrupts while we read `NUM_OVERFLOWS_16384_CYCLE_COUNTER`
        // or we might get an inconsistent value (e.g. in the middle of a
        // write to `NUM_OVERFLOWS_16384_CYCLE_COUNTER` which is a multi-byte
        // value that cannot be written atomically).
        let overflows = without_interrupts_restore(|| NUM_OVERFLOWS_16384_CYCLE_COUNTER );
        let counter_value: u8 = read_volatile(TCNT0);


        // Number of instructions that `micros()` itself takes.
        // Obtained by looking at the generated assembly.
        // TODO Write `micros()` in assembly instead so we know
        //      that this number is correct.
        let own_cycles = 113;

        // Rust doesn't generate the below efficiently.
        // It generates long repetitions of `lsr`, `ror`, `add` and `adc`
        // (up to 14 of each in row).
        // `avr-gcc` seems to generate more efficient code (though not super)
        // sure as it generates loops for those.

        // let cycles = overflows * 16384 + (counter_value as u32) * TIMER_0_PRESCALER_NUM;
        // ~150 cycles

        // Ideally we'd like to write:

        // let cycles = (overflows * 256 + (counter_value as u32)) * TIMER_0_PRESCALER_NUM;
        // return (cycles - own_cycles) / ((CPU_FREQUENCY_HZ / 1_000_000) as u32);
        // ~111 cycles

        // But the multiplication by `TIMER_0_PRESCALER` makes `cycles`
        // overflow quickly; we should rather do the `/ ((CPU_FREQUENCY_HZ / 1_000_000)`
        // division before:

        let cycles_micros =
            (overflows * 256 + (counter_value as u32))
            * (TIMER_0_PRESCALER_NUM / CYCLES_PER_US);
        return cycles_micros - (own_cycles / CYCLES_PER_US);
    }
}

/// A small busy loop.
fn small_delay() {

    for _ in 0..100000 {
        unsafe { asm!("" :::: "volatile")}
    }

}


pub enum InterruptVector {
    ExternalInterruptRequest0,
    ExternalInterruptRequest1,
    PinChangeInterruptRequest0,
    PinChangeInterruptRequest1,
    PinChangeInterruptRequest2,
    WatchdogTimeOutInterrupt,
    TimerCounter2CompareMatchA,
    TimerCounter2CompareMatchB,
    TimerCounter2Overflow,
    TimerCounter1CaptureEvent,
    TimerCounter1CompareMatchA,
    TimerCounter1CompareMatchB,
    TimerCounter1Overflow,
    TimerCounter0CompareMatchA,
    TimerCounter0CompareMatchB,
    TimerCounter0Overflow,
    SPISerialTransferComplete,
    USARTRxComplete,
    USARTDataRegisterEmpty,
    USARTTxComplete,
    ADCConversionComplete,
    EEPROMReady,
    AnalogComparator,
    TwoWireSerialInterface,
    StoreProgramMemoryRead,
}

pub fn vector_number_of(i: InterruptVector) -> u8 {
    match i {
        InterruptVector::ExternalInterruptRequest0 => 1,
        InterruptVector::ExternalInterruptRequest1 => 2,
        InterruptVector::PinChangeInterruptRequest0 => 3,
        InterruptVector::PinChangeInterruptRequest1 => 4,
        InterruptVector::PinChangeInterruptRequest2 => 5,
        InterruptVector::WatchdogTimeOutInterrupt => 6,
        InterruptVector::TimerCounter2CompareMatchA => 7,
        InterruptVector::TimerCounter2CompareMatchB => 8,
        InterruptVector::TimerCounter2Overflow => 9,
        InterruptVector::TimerCounter1CaptureEvent => 10,
        InterruptVector::TimerCounter1CompareMatchA => 11,
        InterruptVector::TimerCounter1CompareMatchB => 12,
        InterruptVector::TimerCounter1Overflow => 13,
        InterruptVector::TimerCounter0CompareMatchA => 14,
        InterruptVector::TimerCounter0CompareMatchB => 15,
        InterruptVector::TimerCounter0Overflow => 16,
        InterruptVector::SPISerialTransferComplete => 17,
        InterruptVector::USARTRxComplete => 18,
        InterruptVector::USARTDataRegisterEmpty => 19,
        InterruptVector::USARTTxComplete => 20,
        InterruptVector::ADCConversionComplete => 21,
        InterruptVector::EEPROMReady => 22,
        InterruptVector::AnalogComparator => 23,
        InterruptVector::TwoWireSerialInterface => 24,
        InterruptVector::StoreProgramMemoryRead => 25,
    }
}


// Macro to define an ISR (Interrupt Service Routine, Interrupt Handler).
//
// Example:
//
//     isr!(TimerCounter1CompareMatchA, {
//       // Reset counter to zero
//       write_volatile(TCNT1, 0);
//     });
//
// Table originally from Arduino's `iom328p.h` file.
macro_rules! isr {
    (ExternalInterruptRequest0,  $e:expr) => { #[no_mangle] pub unsafe extern "avr-interrupt" fn __vector_1()  { $e } };
    (ExternalInterruptRequest1,  $e:expr) => { #[no_mangle] pub unsafe extern "avr-interrupt" fn __vector_2()  { $e } };
    (PinChangeInterruptRequest0, $e:expr) => { #[no_mangle] pub unsafe extern "avr-interrupt" fn __vector_3()  { $e } };
    (PinChangeInterruptRequest1, $e:expr) => { #[no_mangle] pub unsafe extern "avr-interrupt" fn __vector_4()  { $e } };
    (PinChangeInterruptRequest2, $e:expr) => { #[no_mangle] pub unsafe extern "avr-interrupt" fn __vector_5()  { $e } };
    (WatchdogTimeOutInterrupt,   $e:expr) => { #[no_mangle] pub unsafe extern "avr-interrupt" fn __vector_6()  { $e } };
    (TimerCounter2CompareMatchA, $e:expr) => { #[no_mangle] pub unsafe extern "avr-interrupt" fn __vector_7()  { $e } };
    (TimerCounter2CompareMatchB, $e:expr) => { #[no_mangle] pub unsafe extern "avr-interrupt" fn __vector_8()  { $e } };
    (TimerCounter2Overflow,      $e:expr) => { #[no_mangle] pub unsafe extern "avr-interrupt" fn __vector_9()  { $e } };
    (TimerCounter1CaptureEvent,  $e:expr) => { #[no_mangle] pub unsafe extern "avr-interrupt" fn __vector_10() { $e } };
    (TimerCounter1CompareMatchA, $e:expr) => { #[no_mangle] pub unsafe extern "avr-interrupt" fn __vector_11() { $e } };
    (TimerCounter1CompareMatchB, $e:expr) => { #[no_mangle] pub unsafe extern "avr-interrupt" fn __vector_12() { $e } };
    (TimerCounter1Overflow,      $e:expr) => { #[no_mangle] pub unsafe extern "avr-interrupt" fn __vector_13() { $e } };
    (TimerCounter0CompareMatchA, $e:expr) => { #[no_mangle] pub unsafe extern "avr-interrupt" fn __vector_14() { $e } };
    (TimerCounter0CompareMatchB, $e:expr) => { #[no_mangle] pub unsafe extern "avr-interrupt" fn __vector_15() { $e } };
    (TimerCounter0Overflow,      $e:expr) => { #[no_mangle] pub unsafe extern "avr-interrupt" fn __vector_16() { $e } };
    (SPISerialTransferComplete,  $e:expr) => { #[no_mangle] pub unsafe extern "avr-interrupt" fn __vector_17() { $e } };
    (USARTRxComplete,            $e:expr) => { #[no_mangle] pub unsafe extern "avr-interrupt" fn __vector_18() { $e } };
    (USARTDataRegisterEmpty,     $e:expr) => { #[no_mangle] pub unsafe extern "avr-interrupt" fn __vector_19() { $e } };
    (USARTTxComplete,            $e:expr) => { #[no_mangle] pub unsafe extern "avr-interrupt" fn __vector_20() { $e } };
    (ADCConversionComplete,      $e:expr) => { #[no_mangle] pub unsafe extern "avr-interrupt" fn __vector_21() { $e } };
    (EEPROMReady,                $e:expr) => { #[no_mangle] pub unsafe extern "avr-interrupt" fn __vector_22() { $e } };
    (AnalogComparator,           $e:expr) => { #[no_mangle] pub unsafe extern "avr-interrupt" fn __vector_23() { $e } };
    (TwoWireSerialInterface,     $e:expr) => { #[no_mangle] pub unsafe extern "avr-interrupt" fn __vector_24() { $e } };
    (StoreProgramMemoryRead,     $e:expr) => { #[no_mangle] pub unsafe extern "avr-interrupt" fn __vector_25() { $e } };
}


isr!(TimerCounter0Overflow, {

    // serial_println(" --- interrupt");

    NUM_OVERFLOWS_16384_CYCLE_COUNTER += 1;
});

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

