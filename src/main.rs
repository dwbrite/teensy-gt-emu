#![no_std]
#![no_main]

use teensy4_panic as _;
extern crate libm;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [KPP])]
mod app {
    use teensy4_bsp as bsp;
    use bsp::board;
    use bsp::hal::{
        flexpwm::{
            Channel, ClockSelect, LoadMode, Prescaler, Submodule, FULL_RELOAD_VALUE_REGISTER,
            PairOperation,
        },
    };
    use imxrt_log as logging;
    use rtic_monotonics::systick::{Systick, *};
    use teensy4_bsp::hal;

    const PWM_FREQUENCY: u32 = 150_000_000;
    const PWM_PRESCALER: Prescaler = Prescaler::Prescaler1;

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
        pwm: Submodule<1, 3>,
        phase: u16,
        max_duty: i16,
        poller: logging::Poller,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let board::Resources {
            mut gpio2,
            pins,
            usb,
            flexpwm1,
            ..
        } = board::t41(cx.device);

        fill_sine_table();

        let (mut pwm, (mut _sm0, _, _, mut sm3)) = flexpwm1;
        let poller = logging::log::usbd(usb, logging::Interrupts::Enabled).unwrap();

        let led = board::led(&mut gpio2, pins.p13);

        let pwm_freq = 25_000;
        let reload = PWM_FREQUENCY / pwm_freq;
        let half_reload = (reload / 2) as i16;

        sm3.set_debug_enable(true);
        sm3.set_wait_enable(true);
        sm3.set_clock_select(ClockSelect::Ipg);
        sm3.set_prescaler(PWM_PRESCALER);
        sm3.set_pair_operation(PairOperation::Independent);
        sm3.set_load_mode(LoadMode::reload_full());
        sm3.set_load_frequency(1);
        sm3.set_initial_count(&pwm, -half_reload);
        sm3.set_value(FULL_RELOAD_VALUE_REGISTER, half_reload);
        sm3.set_turn_on(Channel::A, -1);
        sm3.set_turn_off(Channel::A, 1);
        sm3.set_output_enable(&mut pwm, Channel::A, true);

        let output_a = hal::flexpwm::Output::new_a(pins.p8);
        // Set the turn on / off count values.
        output_a.set_turn_on(&sm3, i16::MIN / 2);
        output_a.set_turn_off(&sm3, i16::MAX / 2);
        // Enable the PWM output.
        output_a.set_output_enable(&mut pwm, true);

        sm3.set_load_ok(&mut pwm);
        sm3.set_running(&mut pwm, true);

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
                pwm: sm3,
                phase: 0,
                max_duty: reload as i16,
                poller,
            },
        )
    }

    #[task(local = [pwm, phase, max_duty])]
    async fn generate(cx: generate::Context) {
        let phase_step: u16 = 300;

        loop {
            let index = (*cx.local.phase >> 8) as usize;
            let sample = unsafe { SINE_TABLE[index] };

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