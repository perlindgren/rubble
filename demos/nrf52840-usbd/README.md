# `nrf52840-usb`

Per Lindgren (per.lindgren@ltu.se)

USB Support to be. (Also BLE MIDI as below)

## Resources

https://infocenter.nordicsemi.com/pdf/nRF52840_PS_v1.1.pdf 6.35




## Resources

BLE-MIDI-spec 1.0a (2015)

### BLE Service and Characteristics Definitions

The following service and characteristic are defined:
• MIDI Service (UUID: 03B80E5A-EDE8-4B33-A751-6CE34EC4C700)
• MIDI Data I/O Characteristic (UUID: 7772E5DB-3868-4112-A1A9-F2669D106BF3)
• write (encryption recommended, write without response is required)
• read (encryption recommended, respond with no payload)
• notify (encryption recommended)

### Connection Interval

The BLE MIDI device must request a connection interval of 15 ms or less. A lower connection interval is preferred in most applications of MIDI. Connection should be established at the lowest connection interval that is currently supported on both the Central and the Peripheral.

### Initial Connection and Pairing

The Central will attempt to read the MIDI I/O characteristic of the Peripheral after establishing a connection with the accessory. The accessory shall respond to the initial MIDI I/O characteristic read with a packet that has no payload.

### Maximum Transmission Unit Negotiation

The accessory must support MTU negotiation and must support the MTU Exchange command.

### Packet Encoding

Unlike legacy MIDI, BLE is a packet based protocol. Incoming messages cannot be instantly forwarded to the receiving party. Instead they must be buffered and transmitted each BLE connection interval, which is negotiated between the sender and receiver. To maintain precise inter-event timing, this protocol uses 13-bit millisecond-resolution timestamps to express the render time and event spacing of MIDI messages.

## Debugging

### Wireshark

### log

``` shell
$ stty -F /dev/ttyXXX speed 1000000 && cat /dev/ttyXXX > log.txt
```

### bluez debug messages

``` shell
$ enable debugging by adding a -d after
$ ExecStart=/usr/libexec/bluetooth/bluetoothd
```

in `/usr/lib/systemd/system/bluetooth.service`

Save, then:

``` shell
$ systemctl daemon-reload
$ systemctl restart bluetooth
```

or alternatively:

``` shell
$ journalctl --unit=bluetooth -f
```

### Midi on arch

``` shell
$ aconnect -i
client 0: 'System' [type=kernel]
    0 'Timer           '
    1 'Announce        '
client 14: 'Midi Through' [type=kernel]
    0 'Midi Through Port-0'
client XXX: 'BLE MIDI' [type=user,pid=242118]
    0 'BLE MIDI Bluetooth'

```

``` shell
$ aseqdump -p XXX
138:0   Note on                 0, note 100, velocity 100
138:0   Note off                0, note 100, velocity 100
138:0   Note on                 0, note 100, velocity 100
138:0   Note off                0, note 100, velocity 100
138:0   Note on                 0, note 100, velocity 100
138:0   Note off                0, note 100, velocity 100
138:0   Note on                 0, note 100, velocity 100
```

### programming

``` shell
$ cd demos
$ openocd -f openocd_jlink.cfg
```

``` shell
$ cd demos/nrf52832-midi
$ cargo run
```
