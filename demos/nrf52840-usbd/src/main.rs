#![no_std]
#![no_main]
#![warn(rust_2018_idioms)]

// We need to import this crate explicitly so we have a panic handler
use panic_semihosting as _;

mod logger;

use {
    bbqueue::Consumer,
    byteorder::{ByteOrder, LittleEndian},
    core::convert::TryInto,
    core::fmt::Write,
    cortex_m::iprintln,
    cortex_m_semihosting::hprintln,
    nrf52840_hal::{
        self as hal,
        gpio::Level,
        nrf52840_pac::{self as pac, UARTE0},
        prelude::*,
        uarte::{Baudrate, Parity, Uarte},
    },
    rtfm::app,
    rubble::{
        att::Handle,
        config::Config,
        gatt_midi::MidiServiceAttrs,
        l2cap::{BleChannelMap, L2CAPState},
        link::{
            ad_structure::AdStructure,
            queue::{PacketQueue, SimpleConsumer, SimpleProducer, SimpleQueue},
            AddressKind, DeviceAddress, LinkLayer, Responder, MIN_PDU_BUF,
        },
        security::NoSecurity,
        time::{Duration, Timer},
    },
    rubble_nrf52::{
        radio::{BleRadio, PacketBuffer},
        timer::BleTimer,
    },
};

pub enum AppConfig {}

impl Config for AppConfig {
    type Timer = BleTimer<pac::TIMER0>;
    type Transmitter = BleRadio;
    type ChannelMapper = BleChannelMap<MidiServiceAttrs, NoSecurity>;

    type PacketQueue = &'static mut SimpleQueue;
    type PacketProducer = SimpleProducer<'static>;
    type PacketConsumer = SimpleConsumer<'static>;
}

// Midi BLE Device Application
const NOTE_DEMO: bool = false;

#[app(device = nrf52840_hal::nrf52840_pac)]
const APP: () = {
    static mut BLE_TX_BUF: PacketBuffer = [0; MIN_PDU_BUF];
    static mut BLE_RX_BUF: PacketBuffer = [0; MIN_PDU_BUF];
    static mut TX_QUEUE: SimpleQueue = SimpleQueue::new();
    static mut RX_QUEUE: SimpleQueue = SimpleQueue::new();
    static mut BLE_LL: LinkLayer<AppConfig> = ();
    static mut BLE_R: Responder<AppConfig> = ();
    static mut RADIO: BleRadio = ();
    static mut SERIAL: Uarte<UARTE0> = ();
    static mut LOG_SINK: Consumer = ();

    #[init(resources = [BLE_TX_BUF, BLE_RX_BUF, TX_QUEUE, RX_QUEUE])]
    fn init() {
        hprintln!("\n<< INIT >>\n").ok();

        let stim = &mut core.ITM.stim[0];

        iprintln!(stim, "Hello, world!");
        hprintln!("\n<< AFTER I >>\n").ok();

        {
            // On reset the internal high frequency clock is used, but starting the HFCLK task
            // switches to the external crystal; this is needed for Bluetooth to work.

            device
                .CLOCK
                .tasks_hfclkstart
                .write(|w| unsafe { w.bits(1) });
            while device.CLOCK.events_hfclkstarted.read().bits() == 0 {}
        }
        hprintln!("\n<< CLOCK SETUP >>\n").ok();

        iprintln!(stim, "Hello, world!");

        loop {}

        let ble_timer = BleTimer::init(device.TIMER0);
        let p0 = device.P0.split();

        let mut serial = {
            let rxd = p0.p0_11.into_floating_input().degrade();
            let txd = p0.p0_05.into_push_pull_output(Level::Low).degrade();

            let pins = hal::uarte::Pins {
                rxd,
                txd,
                cts: None,
                rts: None,
            };

            device
                .UARTE0
                .constrain(pins, Parity::EXCLUDED, Baudrate::BAUD1M)
        };
        writeln!(serial, "\n--- INIT ---").unwrap();

        let mut devaddr = [0u8; 6];
        let devaddr_lo = device.FICR.deviceaddr[0].read().bits();
        let devaddr_hi = device.FICR.deviceaddr[1].read().bits() as u16;
        LittleEndian::write_u32(&mut devaddr, devaddr_lo);
        LittleEndian::write_u16(&mut devaddr[4..], devaddr_hi);

        let devaddr_type = if device
            .FICR
            .deviceaddrtype
            .read()
            .deviceaddrtype()
            .is_public()
        {
            AddressKind::Public
        } else {
            AddressKind::Random
        };
        let device_address = DeviceAddress::new(devaddr, devaddr_type);

        let mut radio = BleRadio::new(device.RADIO, resources.BLE_TX_BUF, resources.BLE_RX_BUF);

        let log_sink = logger::init(ble_timer.create_stamp_source());

        // Create TX/RX queues
        let (tx, tx_cons) = resources.TX_QUEUE.split();
        let (rx_prod, rx) = resources.RX_QUEUE.split();

        // Create the actual BLE stack objects
        let mut ll = LinkLayer::<AppConfig>::new(device_address, ble_timer);

        let resp = Responder::new(
            tx,
            rx,
            L2CAPState::new(BleChannelMap::with_attributes(MidiServiceAttrs::new())),
        );

        // Send advertisement and set up regular interrupt
        let next_update = ll
            .start_advertise(
                Duration::from_millis(200),
                &[AdStructure::CompleteLocalName("BLE MIDI")],
                &mut radio,
                tx_cons,
                rx_prod,
            )
            .unwrap();
        ll.timer().configure_interrupt(next_update);

        RADIO = radio;
        BLE_LL = ll;
        BLE_R = resp;
        SERIAL = serial;
        LOG_SINK = log_sink;
    }

    #[interrupt(resources = [RADIO, BLE_LL], spawn = [ble_worker])]
    fn RADIO() {
        if let Some(cmd) = resources
            .RADIO
            .recv_interrupt(resources.BLE_LL.timer().now(), &mut resources.BLE_LL)
        {
            resources.RADIO.configure_receiver(cmd.radio);
            resources
                .BLE_LL
                .timer()
                .configure_interrupt(cmd.next_update);

            if cmd.queued_work {
                // If there's any lower-priority work to be done, ensure that happens.
                // If we fail to spawn the task, it's already scheduled.
                spawn.ble_worker().ok();
            }
        }
    }

    #[interrupt(resources = [RADIO, BLE_LL],  spawn = [ble_worker])]
    fn TIMER0() {
        let timer = resources.BLE_LL.timer();
        if !timer.is_interrupt_pending() {
            return;
        }
        timer.clear_interrupt();

        let cmd = resources.BLE_LL.update_timer(&mut *resources.RADIO);
        resources.RADIO.configure_receiver(cmd.radio);

        resources
            .BLE_LL
            .timer()
            .configure_interrupt(cmd.next_update);

        if cmd.queued_work {
            // If there's any lower-priority work to be done, ensure that happens.
            // If we fail to spawn the task, it's already scheduled.
            spawn.ble_worker().ok();
        }
    }

    #[idle(resources = [LOG_SINK, SERIAL, BLE_R, BLE_LL])]
    fn idle() -> ! {
        let mut on = true;
        let mut time = 0;
        // Drain the logging buffer through the serial connection
        loop {
            if cfg!(feature = "log") {
                while let Ok(grant) = resources.LOG_SINK.read() {
                    for chunk in grant.buf().chunks(255) {
                        resources.SERIAL.write(chunk).unwrap();
                    }

                    resources.LOG_SINK.release(grant.buf().len(), grant);
                }
            }

            if resources.BLE_LL.lock(|ble_ll| ble_ll.is_connected()) {
                resources.BLE_R.lock(|ble_r| {
                    match ble_r.l2cap().att() {
                        Some(att) => {
                            if NOTE_DEMO {
                                att.notify_raw(
                                    Handle::from_raw(0x0003),
                                    &MidiPkg::new(
                                        time,
                                        match on {
                                            true => 0x90, // note on, channel 0
                                            _ => 0x80,    // note off, channel 0
                                        },
                                        0x64,
                                        0x64,
                                    )
                                    .0,
                                )
                            } else {
                                att.notify_raw(
                                    Handle::from_raw(0x0003),
                                    &MidiPkg::new(
                                        time,
                                        0xb0, // contoller on channel 1
                                        0x05, // breath controller
                                        match on {
                                            true => 0x7f, // note on, channel 0
                                            _ => 0x00,    // note off, channel 0
                                        },
                                    )
                                    .0,
                                )
                            }
                            on = !on;
                            time += 1; // this shoud be time in micro seconds
                        }
                        _ => (),
                    }
                });
            }
        }
    }
    #[task(resources = [BLE_R])]
    fn ble_worker() {
        // Fully drain the packet queue
        while resources.BLE_R.has_work() {
            resources.BLE_R.process_one().unwrap();
        }
    }

    extern "C" {
        fn WDT();
    }
};

struct MidiPkg([u8; 5]);

impl MidiPkg {
    fn new(time: u32, status: u8, md1: u8, md2: u8) -> Self {
        let mut pkg = MidiPkg([0; 5]);
        pkg.set(time, status, md1, md2);
        pkg
    }

    fn set(&mut self, time: u32, status: u8, md1: u8, md2: u8) {
        self.0[0] = ((1 << 7) | ((time >> 7) & 0b0011_1111)).try_into().unwrap();
        self.0[1] = ((1 << 7) | (time & 0b0111_1111)).try_into().unwrap();
        self.0[2] = status;
        self.0[3] = md1;
        self.0[4] = md2;
    }

    pub fn control_msg(&mut self, time: u32, chan: u8, ctrl_no: u8, ctrl_val: u8) {
        self.set(
            time,
            0xb0 | (chan | 0x0f), // control message
            ctrl_no & 0x7f,       // controller number
            ctrl_val & 0x7f,      // controller value
        )
    }
}
