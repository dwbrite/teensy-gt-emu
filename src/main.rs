#![no_std]
#![no_main]
#![feature(lang_items)]

extern crate libm;
extern crate alloc;
extern crate teensy4_panic;

use gametank_emulator_core::emulator::TimeDaemon;
use linked_list_allocator::LockedHeap;
use rtic_monotonics::Monotonic;
use rtic_monotonics::systick::Systick;
use teensy4_panic as _;

use core::alloc::{GlobalAlloc, Layout};
use core::ptr::null_mut;
use linked_list_allocator::Heap;

struct Rtc;

#[global_allocator]
static GLOBAL_ALLOCATOR: LockedHeap = LockedHeap::empty();

impl TimeDaemon for Rtc {
    fn get_now_ms(&self) -> f64 {
        Systick::now().ticks() as f64
    }
}

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [KPP])]
mod app {
    use core::ptr::write_volatile;use teensy4_bsp as bsp;
    use bsp::board;
    use gametank_emulator_core::cartridges::CartridgeType;
    use gametank_emulator_core::cartridges::CartridgeType::Cart8k;
    use gametank_emulator_core::emulator::{Emulator, PlayState};
    use imxrt_log as logging;
    use log::info;
    use rtic_monotonics::systick::{Systick, *};
    use rtrb::Consumer;
    use teensy4_bsp::hal::dma::channel::Configuration;
    use teensy4_bsp::ral::gpio::{GPIO2};
    use crate::{Rtc, GLOBAL_ALLOCATOR};

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
        phase: u16,
        poller: logging::Poller,
        emulator: Emulator<Rtc>,
        audio_consumer: Consumer<u8>,
    }

    const fn generate_gpio_words() -> [u32; 256] {
        let mut table = [0u32; 256];
        let mut i = 0;
        while i < 256 {
            table[i] = {
                let sample = i as u8;
                let mut word = 0;
                if sample & 0b0000_0001 != 0 { word |= 1 << 10; }
                if sample & 0b0000_0010 != 0 { word |= 1 << 17; }
                if sample & 0b0000_0100 != 0 { word |= 1 << 16; }
                if sample & 0b0000_1000 != 0 { word |= 1 << 11; }
                if sample & 0b0001_0000 != 0 { word |= 1 << 00; }
                if sample & 0b0010_0000 != 0 { word |= 1 << 02; }
                if sample & 0b0100_0000 != 0 { word |= 1 << 01; }
                if sample & 0b1000_0000 != 0 { word |= 1 << 03; }
                word
            };
            i += 1;
        }
        table
    }

    static LOOKUP_TABLE: [u32; 256] = generate_gpio_words();

    static mut GPIO_DMA_SAMPLE_SRC: u32 = 0;

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let board::Resources {
            mut gpio2,
            pins,
            usb,
            mut dma,
            ..
        } = board::t41(cx.device);

        unsafe {
            let heap_start = imxrt_rt::heap_start() as usize;
            let heap_size = 640 * 1024;
            GLOBAL_ALLOCATOR.lock().init(heap_start, heap_size);
        }

        let _ = gpio2.output(pins.p6);
        let _ = gpio2.output(pins.p7);
        let _ = gpio2.output(pins.p8);
        let _ = gpio2.output(pins.p9);
        let _ = gpio2.output(pins.p10);
        let _ = gpio2.output(pins.p11);
        let _ = gpio2.output(pins.p12);
        let _ = gpio2.output(pins.p13);

        fill_sine_table();

        let mut dma0 = dma[0].take().unwrap();
        unsafe {
            let src_ptr = &raw const GPIO_DMA_SAMPLE_SRC;
            let dr_ptr = &(*GPIO2).DR as *const _ as *mut u32 ;


            dma0.reset();

            dma0.set_source_address(src_ptr);
            dma0.set_destination_address(dr_ptr);
            dma0.set_source_offset(0);
            dma0.set_destination_offset(0);

            dma0.set_minor_loop_bytes(4);
            dma0.set_transfer_iterations(0xFFFF); // arbitrarily large — fires continuously

            dma0.set_source_attributes::<u32>(0);
            dma0.set_destination_attributes::<u32>(0);

            dma0.set_source_last_address_adjustment(0);
            dma0.set_destination_last_address_adjustment(0);

            dma0.set_disable_on_completion(false); // don't halt
            dma0.set_interrupt_on_completion(false); // don't interrupt
            dma0.set_channel_configuration(Configuration::AlwaysOn);

            dma0.enable(); // enable ERQ
        }

        let active = dma0.is_active();
        let enabled = dma0.is_enabled();
        let signal = dma0.is_hardware_signaling();

        let poller = logging::log::usbd(usb, logging::Interrupts::Enabled).unwrap();

        Systick::start(
            cx.core.SYST,
            board::ARM_FREQUENCY,
            rtic_monotonics::create_systick_token!(),
        );


        let rtc = Rtc;
        let (producer, audio_consumer) = rtrb::RingBuffer::new(1024); // 1K samples, just in case
        let mut emulator = Emulator::init(rtc, producer);

        info!("pre-emulator");

        // generate::spawn().unwrap();
        core_loop::spawn().unwrap();

        // led.toggle();

        (
            Shared {},
            Local {
                phase: 0,
                poller,
                emulator,
                audio_consumer,
                // led,
            },
        )
    }

    #[task(local = [emulator, audio_consumer])]
    async fn core_loop(cx: core_loop::Context) {
        let emu = cx.local.emulator;
        let audio = cx.local.audio_consumer;
        Systick::delay(2000.millis()).await;

        emu.play_state = PlayState::Playing;

        loop {
            emu.process_cycles(false);
            while !audio.is_empty() {
                if let Ok(word) = audio.pop() {
                }
            }

            Systick::delay(100.micros()).await;
        }
    }

    // #[task(local = [phase])]
    // async fn generate(cx: generate::Context) {
    //     let phase_step: u16 = 20;
    //
    //     loop {
    //         let index = (*cx.local.phase >> 8) as usize;
    //         let sample = unsafe { SINE_TABLE[index] };
    //         let word = LOOKUP_TABLE[sample as usize];
    //         unsafe {
    //             write_volatile(&raw mut GPIO_DMA_SAMPLE_SRC, word);
    //         }
    //
    //         *cx.local.phase = cx.local.phase.wrapping_add(phase_step);
    //         Systick::delay(100.micros()).await;
    //     }
    // }

    #[task(binds = USB_OTG1, local = [poller])]
    fn log_over_usb(cx: log_over_usb::Context) {
        cx.local.poller.poll();
    }
}
