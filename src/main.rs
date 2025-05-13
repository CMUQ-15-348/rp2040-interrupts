/* A simple demo for interrupt handling.
 * Connect a push button switch between GPIO15 and power.
 * (This program configures a pull down resistor for GPIO15.)
 * Push the switch, the LED toggles.
 * This program does not handle debouncing, so you may see some weirdness.
 */
#![no_main]
#![no_std]

use cortex_m;
use cortex_m_rt::entry;
use rp2040_boot2;

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use core::ptr::{read_volatile, write_volatile};

/* The stage 2 bootloader */
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

/* The vector table. We don't have a good way to only change one handler, so
 * we are defining the entire table. They are all the default handlers except
 * for number 13, which is declared below.
 * We declare the default handler as an extern below. The linker sets it up
 * thanks to the device.x file.
 */
#[link_section = ".vector_table.interrupts"]
#[no_mangle]
pub static __INTERRUPTS: [unsafe extern "C" fn(); 26] = [
    DefHandler,   //0
    DefHandler,   //1
    DefHandler,   //2
    DefHandler,   //3
    DefHandler,   //4
    DefHandler,   //5
    DefHandler,   //6
    DefHandler,   //7
    DefHandler,   //8
    DefHandler,   //9
    DefHandler,   //10
    DefHandler,   //11
    DefHandler,   //12
    IO_IRQ_BANK0, //13
    DefHandler,   //14
    DefHandler,   //15
    DefHandler,   //16
    DefHandler,   //17
    DefHandler,   //18
    DefHandler,   //19
    DefHandler,   //20
    DefHandler,   //21
    DefHandler,   //22
    DefHandler,   //23
    DefHandler,   //24
    DefHandler,   //25
];

extern "C" {
    fn DefHandler();
}

/* Some helper functions to directly read/write registers.
 * The are unsafe because they dereference raw pointers.
 * The are volatile because the compiler should not optimize them away.
 */
pub fn read_reg(addr: u32) -> u32 {
    unsafe { read_volatile(addr as *const u32) }
}

pub fn write_reg(addr: u32, value: u32) {
    unsafe {
        write_volatile(addr as *mut u32, value);
    }
}

pub fn set_bits(addr: u32, mask: u32) {
    unsafe {
        write_volatile((addr + 0x2000) as *mut u32, mask);
    }
}

pub fn clear_bits(addr: u32, mask: u32) {
    unsafe {
        write_volatile((addr + 0x3000) as *mut u32, mask);
    }
}

pub const RESETS_BASE: u32 = 0x4000_c000_u32;
pub const PADS_BANK0_BASE: u32 = 0x4001_c000_u32;
pub const IO_BANK0_BASE: u32 = 0x4001_4000_u32;
pub const SIO_BASE: u32 = 0xd000_0000_u32;
pub const NVIC_ISER: u32 = 0xe000_e100_u32;

/*
 * Code to initialize the pins/pads
 */
fn init_io(output_pin: u32, input_pin: u32) {
    // Reset, then deassert the reset on IO_BANK0
    // See Section 2.14 in the datasheet for details
    set_bits(RESETS_BASE, 1 << 5); // Write 1 to reset
    clear_bits(RESETS_BASE, 1 << 5); // Write 0 to deassert reset

    // Reset, then deassert the reset on PADS_BANK0
    // See Section 2.14 in the datasheet for details
    set_bits(RESETS_BASE, 1 << 8); // Write 1 to reset
    clear_bits(RESETS_BASE, 1 << 8); // Write 0 to deassert reset

    // Output pin: Pad configuration.
    // Writing 0 disables input and enables output for that pad.
    // See Table 339 and Table 341 in the datasheet for details
    write_reg(PADS_BANK0_BASE + (output_pin + 1) * 4, 0);

    // Output pin: Setup pin for SIO.
    // Configure IO_BANK0: Set GPIO??_CTRL.funcsel = 5, which selects SIO control.
    // The IO_BANK0 peripheral base address is 0x4001_4000. According to the datasheet,
    // each GPIO has 8 bytes of registers. For example, the GPIO15 CTRL register is located at:
    //   offset = (15 * 8) + 4 = 124 (0x7C)
    // See Table 283, Table 285, and Table 279 in the datasheet for details
    write_reg(IO_BANK0_BASE + (output_pin * 8 + 4), 5);

    // Output pin: Enable SIO output
    // The SIO peripheral base address is 0xD000_0000.
    // The GPIO_OE_SET register is at offset 0x024.
    // We first need to enable the output driver for GPIO??.
    // See Table 16 and Table 25 in the datasheet for details
    set_bits(SIO_BASE, 1 << output_pin);
    set_bits(SIO_BASE + 0x024, 1 << output_pin);

    // Input pin: Pad configuration
    // Output disable on the pad
    set_bits(PADS_BANK0_BASE + (input_pin + 1) * 4, 1 << 7);
    // Enable input on the pad
    set_bits(PADS_BANK0_BASE + (input_pin + 1) * 4, 1 << 6);
    // Select sio function
    write_reg(IO_BANK0_BASE + (input_pin * 8 + 4), 5);
    // Disable output enable for GPIO in SIO
    // The GPIO_OE_CLR register is at offset 0x028.
    set_bits(SIO_BASE + 0x028, 1 << input_pin);
    // Enable pulldown on the pad
    set_bits(PADS_BANK0_BASE + (input_pin + 1) * 4, 1 << 2);
    // Disable pullup
    clear_bits(PADS_BANK0_BASE + (input_pin + 1) * 4, 1 << 3);
}

const OUTPUT_PIN: u32 = 25;
const INPUT_PIN: u32 = 15;

#[entry]
fn main() -> ! {
    // Take peripherals out of reset (IO_BANK0, PADS_BANK0)
    init_io(OUTPUT_PIN, INPUT_PIN);

    // Enable rising edge interrupt on GPIO15 using the INTE register
    write_reg(IO_BANK0_BASE + 0x104, 1 << 31);

    // Clear any pending rising edge using the INTR register
    write_reg(IO_BANK0_BASE + 0x0f4, 1 << 31);

    // Enable IO_BANK0 interrupt in NVIC (interrupt 13)
    write_reg(NVIC_ISER, 1 << 13);

    // Turn LED "off": Clear GPIO?? high. The GPIO_OUT_CLR register is at offset 0x018.
    // See Table 16 and Table 21 in the datasheet for details
    info!("LED off");
    write_reg(SIO_BASE + 0x018, 1 << OUTPUT_PIN);

    // Main loop: Just wait for interrupts.  You could do other things here instead, but this
    // demo doesn't have anything to do.
    loop {
        cortex_m::asm::wfi();
    }
}

// Interrupt handler for IO_BANK0 (GPIO interrupts)
#[allow(non_snake_case)]
#[no_mangle]
pub extern "C" fn IO_IRQ_BANK0() {
    // Check: Was this an interrupt from pin 15?
    let status = read_reg(IO_BANK0_BASE + 0x124);
    if (status & (1 << 31)) != 0 {
        info!("LED Toggle");
        // Toggle the output pin
        write_reg(SIO_BASE + 0x1c, 1 << OUTPUT_PIN);
        // Clear the interrupt
        write_reg(IO_BANK0_BASE + 0x0f4, 1 << 31); // Clear falling edge event
    }
}
