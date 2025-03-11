rustup override set nightly
rustup target add thumbv7em-none-eabihf           
rustup component add llvm-tools-preview
cargo install cargo-binutils
cargo install just

# install teensy loader cli, elsewhere
sudo dnf install libusb-compat-0.1-devel
git clone git@github.com:PaulStoffregen/teensy_loader_cli
cd teensy_loader_cli
make
ln -s ${thatdir}/teensy_loader_cli ~/.local/bin/teensy_loader_cli
