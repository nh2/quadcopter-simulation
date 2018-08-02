# Note the Arduino Nano v3 (with CH340D) needs "-b 57600" to avrdude, it doesn't work with default params
PORT=/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0
EXTRA_AVRDUDE_ARGS="-b 57600"

# PORT=/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_55736313737351A0A291-if00

.PHONY: upload
upload: target/avr-atmega328p/release/blink.hex
	avrdude -p atmega328p -c arduino -P $(PORT) -U flash:w:target/avr-atmega328p/release/blink.hex:i $(EXTRA_AVRDUDE_ARGS)
	# /home/niklas/opt/arduino/arduino-1.8.5/hardware/tools/avr/bin/avrdude "/home/niklas/opt/arduino/arduino-1.8.5/hardware/tools/avr/bin/avrdude" "-C/home/niklas/opt/arduino/arduino-1.8.5/hardware/tools/avr/etc/avrdude.conf" "-q" "-q" "-patmega328p" "-carduino" "-P/dev/ttyUSB0" "-b57600" "-D" "-Uflash:w:/tmp/arduino_build_589937/accel3-double-aref-serial.ino.hex:i"

target/avr-atmega328p/release/blink.hex: target/avr-atmega328p/release/blink.elf
	avr-objcopy -O ihex -R .eeprom target/avr-atmega328p/release/blink.elf target/avr-atmega328p/release/blink.hex

target/avr-atmega328p/release/blink.elf: src/main.rs
	XARGO_RUST_SRC=/home/niklas/src/avr-rust/rust/ RUST_TARGET_PATH=$(shell pwd) /home/niklas/.cargo/bin/rustup run avr-toolchain xargo build --target avr-atmega328p --release
