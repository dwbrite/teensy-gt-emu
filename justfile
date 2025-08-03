build:
    cargo build --release
    cargo objcopy --release -- -O ihex teensy-gt-emu.hex

flash: build
    - tycmd upload -w --nocheck --noreset teensy-gt-emu.hex
    tycmd upload -w --nocheck --noreset teensy-gt-emu.hex
    - tycmd reset
    sudo tycmd monitor
