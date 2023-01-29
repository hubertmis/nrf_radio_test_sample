#![no_main]
#![cfg_attr(not(test), no_std)]

#[cfg(not(test))]
use defmt_rtt as _;

use nrf52840_hal::pac::{CorePeripherals, Peripherals};
use nrf52840_hal::temp::Temp;
use nrf_radio::radio::{self, Phy};
use nrf_radio::error::Error;
use nrf_radio::frame_buffer::frame_allocator::FrameAllocator;
use nrf_radio::frame_buffer::single_frame_allocator::SingleFrameAllocator;

#[cfg(not(test))]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    defmt::info!("panicked");
    exit()
}

pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    defmt::info!("Hello rust world!");
    let peripherals = Peripherals::take().unwrap();
    let mut temp_sensor = Temp::new(peripherals.TEMP);
    let die_temp_c: i32 = temp_sensor.measure().to_num();
    defmt::info!("processor temp is {}°C", die_temp_c);

    let p0 = peripherals.P0;
    p0.dir.write(|w| w.pin13().set_bit());

    // Enable hfxo
    nrf52840_hal::clocks::Clocks::new(peripherals.CLOCK).enable_ext_hfosc();

    // Enable radio IRQs
    // TODO: this procedure should be offloaded to some IRQ management module
    let mut core_periphs = CorePeripherals::take().unwrap();
    let radio_irq_num = nrf52840_hal::pac::Interrupt::RADIO;
    unsafe {
        core_periphs.NVIC.set_priority(radio_irq_num, 0);
        nrf52840_hal::pac::NVIC::unmask(radio_irq_num);
    }

    let frames_allocator = SingleFrameAllocator::new();

    let phy = Phy::new(&peripherals.RADIO);
    phy.configure_802154();
    phy.set_channel(26).unwrap();
    let mut frame: [u8; 17] = [16, 0x41, 0x98, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xcd, 0xab,
                                   0x01, 0x02, 0x03, 0x04, 0x05,
                                   0x00, 0x00];
    loop {
        frame[14] += 1;
        //phy.tx(&frame, tx_cb, &None::<u8>);
        {
            let frame = frames_allocator.get_frame();
            if let Ok(frame) = frame {
                phy.rx(frame, rx_cb).unwrap();
            } else {
                defmt::info!("No buffers available");
            }
        }
        p0.out.write(|w| w.pin13().set_bit());
        wait();
        p0.out.write(|w| w.pin13().clear_bit());
        wait();
    }
}

use core::any::Any;
fn tx_cb(result: Result<(), Error>, context: &'static (dyn Any + Send + Sync)) {
    if result.is_ok() {
        defmt::info!("Tx done");
        let x: Option<&Option<u8>> = context.downcast_ref();
        let x = x.unwrap();
        defmt::info!("context: {}", x);
    } else {
        defmt::info!("Tx error");
    }
}

fn rx_cb(result: Result<radio::RxOk, Error>) {
    if let Ok(result) = result {
        defmt::info!("Received frame: {:x}", result.frame);
    } else {
        defmt::info!("Rx error");
    }
}

#[cfg(test)]
fn wait() {
}

#[cfg(not(test))]
fn wait() {
    for _ in 0..1000000 {
        cortex_m::asm::nop();
    }
}
