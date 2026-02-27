# can-imu-fw

Rust project to to read two IMUs, do some state estimation, and put Heave, Pitch, and Roll estimates
onto a CAN bus in a NMEA2000 compatible way. Also supports CANOpen for configuration.

## Repository Layout

- `canimu`: The main IMU application
- `usb-serial-bridge`: An application to forward the 3DM-CV7 UART to a USB serial port to allow
  talking to the IMU from a PC
- `web-client`: A javascript client for displaying data from the IMU using WebUSB.

## IMUs

The main IMU is a Microstrain 3DM-CV7, and there is a secondary ICM20948 on the board, mainly for
comparison purposes.

Both are used for computing pitch/roll and heave (aka vertical displacement) -- the heave output is
z-position, but passed through a high-pass filter.

## NMEA2000 Messages

The device by default outputs two NMEA2000 messages:

- Heave (PGN 127252)
- Attitude (PGN 127257)

## USB Port

The USB-C port serves as a `gs_usb`/candlelight compatible CAN adapter, as well as a DFU interface
for firmware updates. The CAN adapter allows communication with the CAN IMU itself, as well as any
other devices connected to its CAN bus.

## Programming via DFU

### Generating .bin file

You will need cargo binutils: `cargo install cargo-binutils`

Generate the binary file:
`cargo objcopy --release -- -O binary canimu.bin`

### Programming .bin file

Boot the board into DFU mode, by turning on DIP switch labeled "DFU", and powering on.

Program with dfu-util:

```
dfu-util -d 1209:2323,0483:df11 -a 0 -D canimu.bin -s 0x8000000:leave
```

The first VID/PID is for the device, the second is for the ST DFU bootloader.
