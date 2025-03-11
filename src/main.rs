#![no_std]
#![no_main]
#![feature(const_for)]

use teensy4_panic as _;
extern crate libm;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [KPP])]
mod app {
    use teensy4_bsp as bsp;
    use bsp::board;
    use bsp::ral as ral;
    use imxrt_log as logging;
    use rtic_monotonics::systick::{Systick, *};
    use teensy4_bsp::hal;
    use teensy4_bsp::hal::adc::ClockSelect;
    use teensy4_bsp::hal::flexpwm::{LoadMode, PairOperation, FULL_RELOAD_VALUE_REGISTER, Prescaler, Channel, Submodule};

    const PWM_FREQUENCY: u32 = 150_000_000;
    const PWM_PRESCALER: Prescaler = Prescaler::Prescaler1;

    const SINE_TABLE: [u8; 256] = generate_sine_table();

    const fn generate_sine_table() -> [u8; 256] {
        let mut table = [0u8; 256];
        let mut i = 0;
        while i < 256 {
            let radians = i as f32 * 2.0 * core::f32::consts::PI / 256.0;
            let val = libm::sinf(radians);
            table[i] = ((val + 1.0) * 127.5) as u8;
            i += 1;
        }
        table
    }

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        pwm: Submodule<1, 0>,
        phase: u16,
        max_duty: i16,
        poller: logging::Poller,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let board::Resources {
            mut ccm,
            mut pit,
            mut gpio2,
            pins,
            usb,
            flexpwm1,
            ..
        } = board::t41(cx.device);

        let (mut module, mut submodules) = flexpwm1;
        let (mut sm0, _, _, _) = submodules;
        let poller = logging::log::usbd(usb, logging::Interrupts::Enabled).unwrap();

        let _ = board::led(&mut gpio2, pins.p13);
        //
        let pwm_pin = pins.p6.alt::<Alt1>(); // PWM1_A0
        //
        // let mut builder = hal::flexpwm::Pwm::::new(ral::pwm::PWM1, &mut ccm);
        // let (mut module, mut sm0) = builder.build::<1, 0>(Blocking::new(pit.0));

        let pwm_freq = 25_000;
        let reload = PWM_FREQUENCY / pwm_freq;
        let half_reload = (reload / 2) as i16;

        sm0.set_debug_enable(true);
        sm0.set_wait_enable(true);
        sm0.set_clock_select(hal::flexpwm::ClockSelect::Ipg);
        sm0.set_prescaler(PWM_PRESCALER);
        sm0.set_pair_operation(PairOperation::Independent);
        sm0.set_load_mode(LoadMode::reload_full());
        sm0.set_load_frequency(1);
        sm0.set_initial_count(&module, -half_reload);
        sm0.set_value(FULL_RELOAD_VALUE_REGISTER, half_reload);
        sm0.set_turn_on(Channel::A, -1);
        sm0.set_turn_off(Channel::A, 1);
        sm0.set_output_enable(&mut module, Channel::A, true);
        sm0.set_load_ok(&mut module);
        sm0.set_running(&mut module, true);

        pwm_pin.set_output_enable(true);

        Systick::start(
            cx.core.SYST,
            board::ARM_FREQUENCY,
            rtic_monotonics::create_systick_token!(),
        );

        generate::spawn().unwrap();

        (
            Shared {},
            Local {
                pwm: sm0,
                phase: 0,
                max_duty: reload as i16,
                poller,
            },
        )
    }

    #[task(local = [pwm, phase, max_duty])]
    async fn generate(cx: generate::Context) {
        let mut phase_step: u16 = 300;

        loop {
            let index = (*cx.local.phase >> 8) as usize;
            let sample = SINE_TABLE[index];

            let centered = sample as i16 - 128;
            let duty = (centered * *cx.local.max_duty) / 255;
            cx.local.pwm.set_turn_on(Channel::A, -duty);
            cx.local.pwm.set_turn_off(Channel::A, duty);

            *cx.local.phase = cx.local.phase.wrapping_add(phase_step);
            Systick::delay(100.micros()).await;
        }
    }

    #[task(binds = USB_OTG1, local = [poller])]
    fn log_over_usb(cx: log_over_usb::Context) {
        cx.local.poller.poll();
    }
}