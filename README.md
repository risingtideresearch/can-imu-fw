# can-imu-fw

Rust project to to read IMUs, do some state estimation, and put Heave, Pitch, and Roll estimates onto a CAN bus in a NMEA2000 compatible way. Also supports CANOpen for configuration.

Work in progress.


## Programming via DFU

### Generating .bin file

You will need cargo binutils: `cargo install cargo-binutils`

Generate the binary file:
`cargo objcopy --release -- -O binary canimu.bin`

Boot the board into DFU mode, by turning on DIP switch labeled "DFU", and powering on.

Program with dfu-util:

`dfu-util -d 0483:df11 -a 0 -D canimu.bin -s 0x8000000:leave`