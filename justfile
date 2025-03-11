build:
    cargo build --release
    cargo objcopy --release -- -O ihex teensy-gt-emu.hex

flash: build
    teensy_loader_cli -mmcu=TEENSY41 -w teensy-gt-emu.hex
