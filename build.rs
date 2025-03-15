fn main() {
    use imxrt_rt::{Family, FlexRamBanks, Memory, RuntimeBuilder};

    RuntimeBuilder::from_flexspi(Family::Imxrt1060, 1984 * 1024)
        .flexram_banks(FlexRamBanks {
            itcm: (256-32)/32,
            dtcm: (128)/32,
            ocram: (128+32)/32,
        })
        .heap(Memory::Ocram)
        .uninit(Memory::Ocram)
        .heap_size(640 * 1024)
        .stack(Memory::Dtcm)
        .stack_size(12  * 1024)
        // .stack_size_env_override("TEENSY4_STACK_SIZE")
        .vectors(Memory::Dtcm)
        .text(Memory::Itcm)
        .data(Memory::Dtcm)
        .bss(Memory::Dtcm)
        .linker_script_name("t4link.x")
        .build()
        .unwrap();
}
