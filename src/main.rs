#![no_std]
#![no_main]
#![feature(lang_items)]

extern crate libm;
extern crate alloc;
extern crate teensy4_panic;

// use gametank_emulator_core::emulator::TimeDaemon;
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

// impl TimeDaemon for Rtc {
//     fn get_now_ms(&self) -> f64 {
//         Systick::now().ticks() as f64
//     }
// }

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [KPP])]
mod app {
    use core::ptr::write_volatile;use teensy4_bsp as bsp;
    use bsp::board;
    // use gametank_emulator_core::cartridges::CartridgeType;
    // use gametank_emulator_core::cartridges::CartridgeType::Cart8k;
    // use gametank_emulator_core::emulator::{Emulator, PlayState};
    use imxrt_log as logging;
    use log::info;
    use rtic_monotonics::systick::{Systick, *};
    // use rtrb::Consumer;
    use teensy4_bsp::hal;
    use teensy4_bsp::hal::dma::channel::Configuration;
    use teensy4_bsp::ral::gpio::{GPIO1, GPIO2};
    use teensy4_bsp::hal::gpio::Output;
    use teensy4_bsp::pins::{configure, Config, DriveStrength, Hysteresis, OpenDrain, SlewRate, Speed};
    use teensy4_bsp::pins::imxrt_iomuxc::Iomuxc;
    use teensy4_bsp::pins::t41::{P28, P29, P30, P31};
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
        // emulator: Emulator<Rtc>,
        // audio_consumer: Consumer<u8>,
        resx: Output<P28>,
        de: Output<P29>,
        wrx: Output<P30>,
        dcx: Output<P31>,
    }

    const fn databus_gpio() -> [[u32; 256]; 2] {
        let mut tables = [[0u32; 256]; 2];
        let mut i = 0;
        while i < 256 {
            tables[0][i] = {
                let byte = i as u8;
                let mut word = 0;
                if byte & 0b0000_0001 != 0 { word |= 1 << 31; } // pin 27
                if byte & 0b0000_0010 != 0 { word |= 1 << 30; } // pin 26
                if byte & 0b0000_0100 != 0 { word |= 1 << 13; } // pin 25
                if byte & 0b0000_1000 != 0 { word |= 1 << 12; } // pin 24
                if byte & 0b0001_0000 != 0 { word |= 1 << 28; } // pin 38
                if byte & 0b0010_0000 != 0 { word |= 1 << 29; } // pin 39
                if byte & 0b0100_0000 != 0 { word |= 1 << 20; } // pin 40
                if byte & 0b1000_0000 != 0 { word |= 1 << 21; } // pin 41
                word
            };
            tables[1][i] = {
                let byte = i as u8;
                let mut word = 0;
                if byte & 0b0000_0001 != 0 { word |= 1 << 18; } // pin 14
                if byte & 0b0000_0010 != 0 { word |= 1 << 19; } // pin 15
                if byte & 0b0000_0100 != 0 { word |= 1 << 23; } // pin 16
                if byte & 0b0000_1000 != 0 { word |= 1 << 22; } // pin 17
                if byte & 0b0001_0000 != 0 { word |= 1 << 17; } // pin 18
                if byte & 0b0010_0000 != 0 { word |= 1 << 16; } // pin 19
                if byte & 0b0100_0000 != 0 { word |= 1 << 26; } // pin 20
                if byte & 0b1000_0000 != 0 { word |= 1 << 27; } // pin 21
                word
            };
            i += 1;
        }
        tables
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
    static DBI_LOOKUP_TABLE: [[u32; 256]; 2] = databus_gpio();

    #[inline(always)]
    pub fn dma_dbi(data: u16) {
        let p0 = DBI_LOOKUP_TABLE[0][(data & 0xFF) as usize];
        let p1 = DBI_LOOKUP_TABLE[1][((data >> 8) & 0xFF) as usize];
        let word = p0 | p1;
        unsafe {
            write_volatile(&raw mut GPIO_DMA_DBI_SRC, word);
        }
    }

    #[inline(always)]
    async fn write_command<Pw, Pdc>(cmd: u8, wrx: &mut Output<Pw>, dcx: &mut Output<Pdc>) {
        dcx.clear();
        dma_dbi(cmd as u16);
        Systick::delay(1.micros()).await;
        wrx.clear();
        Systick::delay(1.micros()).await;
        wrx.set();
        Systick::delay(1.micros()).await;
    }

    #[inline(always)]
    async fn write_data<Pw, Pdc>(data: u16, wrx: &mut Output<Pw>, dcx: &mut Output<Pdc>) {
        dcx.set();
        dma_dbi(data);
        Systick::delay(1.micros()).await;
        wrx.clear();
        Systick::delay(1.micros()).await;
        wrx.set();
        Systick::delay(1.micros()).await;
    }


    static mut GPIO_DMA_SAMPLE_SRC: u32 = 0;
    static mut GPIO_DMA_DBI_SRC: u32 = 0;

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let board::Resources {
            mut gpio1,
            mut gpio2,
            mut gpio3,
            mut gpio4,
            mut pins,
            usb,
            mut dma,
            ..
        } = board::t41(cx.device);

        unsafe {
            let heap_start = imxrt_rt::heap_start() as usize;
            let heap_size = 640 * 1024;
            GLOBAL_ALLOCATOR.lock().init(heap_start, heap_size);
        }

        let cfg = Config::zero()
            .set_slew_rate(SlewRate::Fast)
            .set_speed(Speed::Max)
            .set_drive_strength(DriveStrength::R0_7)
            .set_open_drain(OpenDrain::Disabled)
            .set_hysteresis(Hysteresis::Disabled)
            .set_pull_keeper(None);

        configure(&mut pins.p6, cfg); configure(&mut pins.p7, cfg); configure(&mut pins.p8, cfg); configure(&mut pins.p9, cfg);
        configure(&mut pins.p10, cfg); configure(&mut pins.p11, cfg); configure(&mut pins.p12, cfg); configure(&mut pins.p13, cfg);

        configure(&mut pins.p27, cfg); configure(&mut pins.p26, cfg); configure(&mut pins.p25, cfg); configure(&mut pins.p24, cfg);
        configure(&mut pins.p38, cfg); configure(&mut pins.p39, cfg); configure(&mut pins.p40, cfg); configure(&mut pins.p41, cfg);

        configure(&mut pins.p14, cfg); configure(&mut pins.p15, cfg); configure(&mut pins.p16, cfg); configure(&mut pins.p17, cfg);
        configure(&mut pins.p18, cfg); configure(&mut pins.p19, cfg); configure(&mut pins.p20, cfg); configure(&mut pins.p21, cfg);


        // audio bus
        let _ = gpio2.output(pins.p6);
        let _ = gpio2.output(pins.p7);
        let _ = gpio2.output(pins.p8);
        let _ = gpio2.output(pins.p9);
        let _ = gpio2.output(pins.p10);
        let _ = gpio2.output(pins.p11);
        let _ = gpio2.output(pins.p12);
        let _ = gpio2.output(pins.p13);

        // video / databus (command + data0)
        let _ = gpio1.output(pins.p27);
        let _ = gpio1.output(pins.p26);
        let _ = gpio1.output(pins.p25);
        let _ = gpio1.output(pins.p24);
        let _ = gpio1.output(pins.p38);
        let _ = gpio1.output(pins.p39);
        let _ = gpio1.output(pins.p40);
        let _ = gpio1.output(pins.p41);
        // video / databus (data1)
        let _ = gpio1.output(pins.p14);
        let _ = gpio1.output(pins.p15);
        let _ = gpio1.output(pins.p16);
        let _ = gpio1.output(pins.p17);
        let _ = gpio1.output(pins.p18);
        let _ = gpio1.output(pins.p19);
        let _ = gpio1.output(pins.p20);
        let _ = gpio1.output(pins.p21);

        // other ctl pins
        configure(&mut pins.p28, cfg); configure(&mut pins.p29, cfg); configure(&mut pins.p30, cfg); configure(&mut pins.p31, cfg);
        let mut resx =  gpio3.output(pins.p28);
        let mut de =    gpio4.output(pins.p29);
        let mut wrx =   gpio3.output(pins.p30);
        let mut dcx =   gpio3.output(pins.p31);

        fill_sine_table();

        let mut dma0 = dma[0].take().unwrap();
        let mut dma1 = dma[1].take().unwrap();


        unsafe {
            let src_ptr = &raw const GPIO_DMA_DBI_SRC;
            let dr_ptr = &(*GPIO1).DR as *const _ as *mut u32 ;


            dma1.reset();

            dma1.set_source_address(src_ptr);
            dma1.set_destination_address(dr_ptr);
            dma1.set_source_offset(0);
            dma1.set_destination_offset(0);

            dma1.set_minor_loop_bytes(4);
            dma1.set_transfer_iterations(0xFFFF); // arbitrarily large — fires continuously

            dma1.set_source_attributes::<u32>(0);
            dma1.set_destination_attributes::<u32>(0);

            dma1.set_source_last_address_adjustment(0);
            dma1.set_destination_last_address_adjustment(0);

            dma1.set_disable_on_completion(false); // don't halt
            dma1.set_interrupt_on_completion(false); // don't interrupt
            dma1.set_channel_configuration(Configuration::AlwaysOn);

            dma1.enable(); // enable ERQ
        }

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


        let poller = logging::log::usbd(usb, logging::Interrupts::Enabled).unwrap();

        Systick::start(
            cx.core.SYST,
            board::ARM_FREQUENCY,
            rtic_monotonics::create_systick_token!(),
        );

        let rtc = Rtc;
        // let (producer, audio_consumer) = rtrb::RingBuffer::new(1024); // 1K samples, just in case
        // let mut emulator = Emulator::init(rtc, producer);

        info!("pre-emulator");

        // generate::spawn().unwrap();
        core_loop::spawn().unwrap();

        // led.toggle();

        (
            Shared {},
            Local {
                phase: 0,
                poller,
                // emulator,
                // audio_consumer,
                // led,
                resx,
                de,
                wrx,
                dcx,
            },
        )
    }

    // ,
    #[task(local = [resx, dcx, wrx])]
    async fn core_loop(cx: core_loop::Context) {
        // let emu = cx.local.emulator;
        // let audio = cx.local.audio_consumer;
        // emu.play_state = PlayState::Playing;

        let mut resx = cx.local.resx;
        let mut dcx = cx.local.dcx;
        let mut wrx = cx.local.wrx;

        resx.clear();
        Systick::delay(500.millis()).await;
        resx.set();
        Systick::delay(120.millis()).await;

        dma_dbi(0xFFFF);
        // Systick::delay(520.millis()).await;
        // unsafe {
        //     for i in 0..256 {
        //         dma_dbi(i);
        //         Systick::delay(6.millis()).await;
        //     }
        // }

        Systick::delay(150.millis()).await;

        write_command(0x01, wrx, dcx).await;

        Systick::delay(150.millis()).await;

        write_command(0x28, wrx, dcx).await; // display off
        write_command(0x3A, wrx, dcx).await; // pixel format
        write_data(0b0000_0101, wrx, dcx).await; // 16 bit pixels for DBI

        // --- Power Control 1 (Vreg1 and Vreg2 voltages) ---
        write_command(0xC0, wrx, dcx).await;
        Systick::delay(1.millis()).await;
        write_data(0x17, wrx, dcx).await; // Vreg1 = 4.7V
        Systick::delay(1.millis()).await;
        write_data(0x15, wrx, dcx).await; // Vreg2 = -4.6V
        Systick::delay(10.millis()).await;

        // --- Power Control 2 (VGH/VGL setting) ---
        write_command(0xC1, wrx, dcx).await;
        Systick::delay(1.millis()).await;
        write_data(0x41, wrx, dcx).await; // 0x41 = default, works fine
        Systick::delay(10.millis()).await;

        // --- VCOM Control (common electrode voltage) ---
        write_command(0xC5, wrx, dcx).await;
        Systick::delay(1.millis()).await;
        write_data(0x00, wrx, dcx).await;
        Systick::delay(1.millis()).await;
        write_data(0x12, wrx, dcx).await;
        Systick::delay(1.millis()).await;
        write_data(0x80, wrx, dcx).await;
        Systick::delay(10.millis()).await;

        // --- Optional: Memory Access Control (MADCTL) ---
        // Sets scan direction and RGB/BGR
        write_command(0x36, wrx, dcx).await;
        Systick::delay(1.millis()).await;
        write_data(0x48, wrx, dcx).await; // idk lol xD
        Systick::delay(10.millis()).await;

        // --- Sleep Out ---
        write_command(0x11, wrx, dcx).await;
        Systick::delay(10.millis()).await;

        // --- Display On ---
        write_command(0x29, wrx, dcx).await;
        Systick::delay(10.millis()).await;

        // --- All Pixels On ---
        write_command(0x23, wrx, dcx).await; // all pixels on

        Systick::delay(500.millis()).await;

        // --- set normal display mode---
        write_command(0x13, wrx, dcx).await;

        Systick::delay(500.millis()).await;

        // write_command(0x51, wrx, dcx).await; // write brightness
        // write_data(0xFF, wrx, dcx).await;

        // // --- All Pixels Off ---
        // write_command(0x22, &mut wrx, &mut dcx).await; // all pixels off


        // write all pixels
        write_command(0x2A, &mut wrx, &mut dcx).await;
        write_data(0x00, &mut wrx, &mut dcx).await; write_data(0x00, &mut wrx, &mut dcx).await; // X start = 0
        write_data(319, &mut wrx, &mut dcx).await; write_data(319, &mut wrx, &mut dcx).await; // X end   = 479 (480)

        write_command(0x2B, &mut wrx, &mut dcx).await;
        write_data(0x00, &mut wrx, &mut dcx).await; write_data(0x00, &mut wrx, &mut dcx).await; // Y start = 0
        write_data(479, &mut wrx, &mut dcx).await; write_data(479, &mut wrx, &mut dcx).await; // Y end   = 319 (320)

        write_command(0x2C, &mut wrx, &mut dcx).await; // Memory write

        // Fill 320*480 in a loop
        for _ in 0..(320*480) {
            write_data(0b00000_111111_00000, &mut wrx, &mut dcx).await;
        }

        loop {
            // Systick::delay(1.micros()).await;
            // resx.toggle();
            // emu.process_cycles(false);
            // while !audio.is_empty() {
            //     if let Ok(word) = audio.pop() {
            //     }
            // }

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
    //
    #[task(binds = USB_OTG1, local = [poller])]
    fn log_over_usb(cx: log_over_usb::Context) {
        cx.local.poller.poll();
    }
}
