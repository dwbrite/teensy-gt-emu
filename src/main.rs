#![no_std]
#![no_main]

use teensy4_panic as _;

extern crate libm;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [KPP])]
mod app {
    use teensy4_bsp as bsp;
    use bsp::board;
    use imxrt_log as logging;
    use rtic_monotonics::systick::{Systick, *};
    use teensy4_bsp::hal::gpio;
    use teensy4_bsp::hal::gpio::Port;
    use teensy4_bsp::ral::gpio::{GPIO1, GPIO2, GPIO3, GPIO4, GPIO5};

    static mut SINE_TABLE: [u8; 256] = [0; 256];

    fn fill_sine_table() {
        use libm::sinf;
        for i in 0..256 {
            let radians = i as f32 * 2.0 * core::f32::consts::PI / 256.0;
            let val = sinf(radians);
            unsafe {
                SINE_TABLE[i] = ((val + 1.0) * 127.5) as u8;
            }
        }
    }

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        p33: gpio::Output<bsp::pins::t41::P33>,
        p34: gpio::Output<bsp::pins::t41::P34>,
        p35: gpio::Output<bsp::pins::t41::P35>,
        p36: gpio::Output<bsp::pins::t41::P36>,
        p37: gpio::Output<bsp::pins::t41::P37>,
        p38: gpio::Output<bsp::pins::t41::P38>,
        p39: gpio::Output<bsp::pins::t41::P39>,
        p40: gpio::Output<bsp::pins::t41::P40>,

        phase: u16,
        poller: logging::Poller,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let board::Resources {
            mut gpio2,
            pins,
            usb,
            ..
        } = board::t41(cx.device);

        fill_sine_table();

        let poller = logging::log::usbd(usb, logging::Interrupts::Enabled).unwrap();
        let led = board::led(&mut gpio2, pins.p13);

        let mut gpio1 = Port::new(unsafe { GPIO1::instance() });
        let mut gpio2 = Port::new(unsafe { GPIO2::instance() });
        // let mut gpio3 = Port::new(unsafe { GPIO3::instance() });
        let mut gpio4 = Port::new(unsafe { GPIO4::instance() });
        // let mut gpio5 = Port::new(unsafe { GPIO5::instance() });

        let p33 = gpio4.output(pins.p33);
        let p34 = gpio2.output(pins.p34);
        let p35 = gpio2.output(pins.p35);
        let p36 = gpio2.output(pins.p36);
        let p37 = gpio2.output(pins.p37);
        let p38 = gpio1.output(pins.p38);
        let p39 = gpio1.output(pins.p39);
        let p40 = gpio1.output(pins.p40);

        Systick::start(
            cx.core.SYST,
            board::ARM_FREQUENCY,
            rtic_monotonics::create_systick_token!(),
        );

        generate::spawn().unwrap();
        led.toggle();

        (
            Shared {},
            Local {
                p33, p34, p35, p36, p37, p38, p39, p40,
                phase: 0,
                poller,
            },
        )
    }

    #[task(local = [p33, p34, p35, p36, p37, p38, p39, p40, phase])]
    async fn generate(cx: generate::Context) {
        let phase_step: u16 = 20;

        loop {
            let index = (*cx.local.phase >> 8) as usize;
            let sample = unsafe { SINE_TABLE[index] };

            if (sample & 0b0000_0001) != 0 { cx.local.p33.set(); } else { cx.local.p33.clear(); }
            if (sample & 0b0000_0010) != 0 { cx.local.p34.set(); } else { cx.local.p34.clear(); }
            if (sample & 0b0000_0100) != 0 { cx.local.p35.set(); } else { cx.local.p35.clear(); }
            if (sample & 0b0000_1000) != 0 { cx.local.p36.set(); } else { cx.local.p36.clear(); }
            if (sample & 0b0001_0000) != 0 { cx.local.p37.set(); } else { cx.local.p37.clear(); }
            if (sample & 0b0010_0000) != 0 { cx.local.p38.set(); } else { cx.local.p38.clear(); }
            if (sample & 0b0100_0000) != 0 { cx.local.p39.set(); } else { cx.local.p39.clear(); }
            if (sample & 0b1000_0000) != 0 { cx.local.p40.set(); } else { cx.local.p40.clear(); }

            *cx.local.phase = cx.local.phase.wrapping_add(phase_step);
            Systick::delay(100.micros()).await;
        }
    }

    #[task(binds = USB_OTG1, local = [poller])]
    fn log_over_usb(cx: log_over_usb::Context) {
        cx.local.poller.poll();
    }
}
