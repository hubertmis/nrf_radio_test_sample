#![no_main]
#![cfg_attr(not(test), no_std)]

#[cfg(not(test))]
use defmt_rtt as _;

use nrf52840_hal::pac::{CorePeripherals, Peripherals};
use nrf52840_hal::temp::Temp;
use nrf_radio::radio::{self, Phy};
use nrf_radio::error::Error;
use nrf_radio::frm_mem_mng::frame_allocator::FrameAllocator;
use nrf_radio::frm_mem_mng::frame_buffer::FrameBuffer;
use nrf_radio::frm_mem_mng::single_frame_allocator::SingleFrameAllocator;
use nrf_radio::ieee802154::pib::Pib;
use nrf_radio::ieee802154::rx::Rx;
use nrf_radio::utils::tasklet::TaskletQueue;

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

static mut PHY: Option<Phy> = None;
static mut TASKLET_QUEUE: Option<TaskletQueue> = None;

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

    static mut PIB: Pib = Pib::new();
    unsafe {
        TASKLET_QUEUE = Some(TaskletQueue::new());
        PHY = Some(Phy::new(&peripherals.RADIO));

        PIB.set_pan_id(&[0x12, 0x34]);
        PIB.set_short_addr(&[0x12, 0x34]);
        PIB.set_ext_addr(&[0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef]);

        let phy_ref = PHY.as_mut().unwrap();
        phy_ref.configure_802154();
        phy_ref.set_channel(26).unwrap();
    }
    let mut frame: [u8; 17] = [16, 0x41, 0x98, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xcd, 0xab,
                                   0x01, 0x02, 0x03, 0x04, 0x05,
                                   0x00, 0x00];
    let rx;
    unsafe {
        rx = Rx::new(PHY.as_ref().unwrap(), &PIB, TASKLET_QUEUE.as_ref().unwrap());
    }
    loop {
        frame[14] += 1;
        //phy.tx(&frame, tx_cb, &None::<u8>);
        {
            let frame = frames_allocator.get_frame();
            if let Ok(frame) = frame {
                let result = rx.start(frame, ieee802154_rx_cb);
                if let Ok(_) = result {
                    defmt::info!("Started RX");
                } else {
                    defmt::info!("Error starting RX");
                }
            } else {
                defmt::info!("No buffers available");
            }
        }
        p0.out.write(|w| w.pin13().set_bit());
        wait();
        p0.out.write(|w| w.pin13().clear_bit());
        wait();
        unsafe {
            TASKLET_QUEUE.as_ref().unwrap().run();
        }
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

fn ieee802154_rx_cb(result: Result<FrameBuffer, Error>) {
    match result {
        Ok(frame) => defmt::info!("Received frame: {:x}", frame),
        Err(Error::NotMatchingPanId) => defmt::info!("Received frame with other Pan Id"),
        Err(Error::NotMatchingAddress) => defmt::info!("Received frame with other destination address"),
        Err(Error::InvalidFrame) => defmt::info!("Received invalid frame"),
        Err(Error::IncorrectCrc) => defmt::info!("Received frame with CRC error"),
        Err(_) => defmt::info!("Unexpected Rx error"),
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
