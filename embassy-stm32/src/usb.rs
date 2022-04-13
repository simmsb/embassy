#![macro_use]

use core::marker::PhantomData;
use core::sync::atomic::{compiler_fence, AtomicU32, Ordering};
use core::task::Poll;
use embassy::interrupt::InterruptExt;
use embassy::time::{Duration, Timer};
use embassy::util::Unborrow;
use embassy::waitqueue::AtomicWaker;
use embassy_hal_common::unborrow;
use embassy_usb::control::Request;
use embassy_usb::driver::{self, EndpointAllocError, EndpointError, Event};
use embassy_usb::types::{EndpointAddress, EndpointInfo, EndpointType, UsbDirection};
use futures::future::poll_fn;
use futures::Future;
use heapless::Vec;

use crate::gpio::low_level::AFType;
use crate::interrupt::Interrupt;
use crate::pac;
use crate::pac::usb::{regs, vals};
use crate::rcc::low_level::RccPeripheral;

const NEW_AW: AtomicWaker = AtomicWaker::new();
static BUS_WAKER: AtomicWaker = NEW_AW;
static EP0_WAKER: AtomicWaker = NEW_AW;
static EP_IN_WAKERS: [AtomicWaker; 8] = [NEW_AW; 8];
static EP_OUT_WAKERS: [AtomicWaker; 8] = [NEW_AW; 8];
static READY_ENDPOINTS: AtomicU32 = AtomicU32::new(0);
static IRQ_FLAGS: AtomicU32 = AtomicU32::new(0);

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct EndpointData {
    ep_addr: u8,
    ep_type: EndpointType,
    used_in: bool,
    used_out: bool,
}

pub struct Driver<'d, T: Instance> {
    phantom: PhantomData<&'d mut T>,
    alloc: Vec<EndpointData, 8>,
}

impl<'d, T: Instance> Driver<'d, T> {
    pub fn new(
        _usb: impl Unborrow<Target = T> + 'd,
        irq: impl Unborrow<Target = T::Interrupt> + 'd,
        dp: impl Unborrow<Target = impl DpPin<T>> + 'd,
        dm: impl Unborrow<Target = impl DmPin<T>> + 'd,
    ) -> Self {
        unborrow!(irq, dp, dm);
        irq.set_handler(Self::on_interrupt);
        irq.unpend();
        irq.enable();

        unsafe {
            dp.set_as_af(dp.af_num(), AFType::OutputPushPull);
            dm.set_as_af(dm.af_num(), AFType::OutputPushPull);
        }

        Self {
            phantom: PhantomData,
            alloc: Vec::new(),
        }
    }

    fn on_interrupt(_: *mut ()) {
        unsafe {
            let regs = T::regs();
            let x = regs.istr().read().0;
            info!("USB IRQ: {:08x}", x);

            // mask of all the enabled irqs
            let mut mask = regs::Istr(0);
            mask.set_wkup(true);
            mask.set_susp(true);
            mask.set_reset(true);

            let istr = regs.istr().read();

            let flags = istr.0 & mask.0;
            if flags != 0 {
                // Ideally we'd disable (mask) the irq and then let main thread check the bit
                // in ISTR, but it's not possible to atomically mask IRQs in CNTR.
                // Instead, send the flags to main thread through an atomic ourselves.
                IRQ_FLAGS.fetch_or(flags, Ordering::AcqRel);

                // Write 0 to clear.
                regs.istr().write_value(regs::Istr(!flags));
            }

            if istr.ctr() {
                let index = istr.ep_id() as usize;
                let mut epr = regs.epr(index).read();
                let mut ready_mask: u32 = 0;
                if epr.ctr_rx() {
                    info!("EP {} RX, setup={}", index, epr.setup());
                    ready_mask |= Out::mask(index);
                }
                if epr.ctr_tx() {
                    info!("EP {} TX", index);
                    ready_mask |= In::mask(index);
                }
                READY_ENDPOINTS.fetch_or(ready_mask, Ordering::AcqRel);
                epr.set_dtog_rx(false);
                epr.set_dtog_tx(false);
                epr.set_stat_rx(vals::StatRx(0));
                epr.set_stat_tx(vals::StatTx(0));
                epr.set_ctr_rx(!epr.ctr_rx());
                epr.set_ctr_tx(!epr.ctr_tx());
                regs.epr(index).write_value(epr);
            }
        }
    }

    fn set_stalled(ep_addr: EndpointAddress, stalled: bool) {
        let regs = T::regs();
        // TODO
    }

    fn is_stalled(ep_addr: EndpointAddress) -> bool {
        let regs = T::regs();
        // TODO
        false
    }

    fn unused_addr(&self) -> Result<u8, EndpointAllocError> {
        for addr in 1..16 {
            if self.alloc.iter().find(|ep| ep.ep_addr == addr).is_none() {
                return Ok(addr);
            }
        }
        Err(EndpointAllocError)
    }

    fn alloc_endpoint<Dir>(
        &mut self,
        ep_addr: Option<EndpointAddress>,
        ep_type: EndpointType,
        max_packet_size: u16,
        interval: u8,
        is_out: bool,
    ) -> Result<Endpoint<'d, T, Dir>, driver::EndpointAllocError> {
        trace!(
            "allocating addr={:?} type={:?} mps={:?} interval={}, out={}",
            ep_addr,
            ep_type,
            max_packet_size,
            interval,
            is_out
        );

        let index = self.alloc.iter_mut().enumerate().find(|(_, ep)| {
            if let Some(ep_addr) = ep_addr {
                // If the user requests a particular addr, check if we already have an EP with that one.
                // If there is, we MUST use that one, it's not allowed to have multiple EPRs with the same addr
                // TODO: is it? the docs say it is for double-buffered.
                ep.ep_addr == ep_addr.index() as _
            } else {
                // Find one EP with the right kind, and where the right half is not used.
                let used = if is_out { ep.used_out } else { ep.used_in };
                ep.ep_type == ep_type && !used
            }
        });

        let index = match index {
            Some((index, _)) => index,
            None => {
                let ep = EndpointData {
                    ep_addr: match ep_addr {
                        Some(ep_addr) => ep_addr.index() as _,
                        None => self.unused_addr()?,
                    },
                    ep_type,
                    used_in: false,
                    used_out: false,
                };

                let index = self.alloc.len();
                self.alloc.push(ep).map_err(|_| EndpointAllocError)?;
                index
            }
        };

        let ep = &mut self.alloc[index];
        trace!("  index={} ep={:?}", index, ep);
        assert!(ep.ep_type == ep_type);
        if let Some(ep_addr) = ep_addr {
            assert!(ep.ep_addr == ep_addr.index() as _);
        }

        if is_out {
            assert!(!ep.used_out);
            ep.used_out = true;
        } else {
            assert!(!ep.used_in);
            ep.used_in = true;
        }

        let ep_addr = EndpointAddress::from_parts(index, UsbDirection::In);
        Ok(Endpoint {
            _phantom: PhantomData,
            index: index as _,
            info: EndpointInfo {
                addr: ep_addr,
                ep_type,
                max_packet_size,
                interval,
            },
        })
    }
}

impl<'d, T: Instance> driver::Driver<'d> for Driver<'d, T> {
    type EndpointOut = Endpoint<'d, T, Out>;
    type EndpointIn = Endpoint<'d, T, In>;
    type ControlPipe = ControlPipe<'d, T>;
    type Bus = Bus<'d, T>;
    type EnableFuture = impl Future<Output = Self::Bus> + 'd;

    fn alloc_endpoint_in(
        &mut self,
        ep_addr: Option<EndpointAddress>,
        ep_type: EndpointType,
        max_packet_size: u16,
        interval: u8,
    ) -> Result<Self::EndpointIn, driver::EndpointAllocError> {
        self.alloc_endpoint(ep_addr, ep_type, max_packet_size, interval, false)
    }

    fn alloc_endpoint_out(
        &mut self,
        ep_addr: Option<EndpointAddress>,
        ep_type: EndpointType,
        max_packet_size: u16,
        interval: u8,
    ) -> Result<Self::EndpointOut, driver::EndpointAllocError> {
        self.alloc_endpoint(ep_addr, ep_type, max_packet_size, interval, true)
    }

    fn alloc_control_pipe(
        &mut self,
        max_packet_size: u16,
    ) -> Result<Self::ControlPipe, driver::EndpointAllocError> {
        self.alloc_endpoint_out(Some(0x00.into()), EndpointType::Control, max_packet_size, 0)?;
        self.alloc_endpoint_in(Some(0x80.into()), EndpointType::Control, max_packet_size, 0)?;
        Ok(ControlPipe {
            _phantom: PhantomData,
            max_packet_size,
        })
    }

    fn enable(self) -> Self::EnableFuture {
        async move {
            let regs = T::regs();

            unsafe {
                crate::peripherals::PWR::enable();

                pac::PWR
                    .cr2()
                    .modify(|w| w.set_usv(pac::pwr::vals::Usv::VALID));

                <T as RccPeripheral>::enable();
                <T as RccPeripheral>::reset();

                regs.cntr().write(|w| {
                    w.set_pdwn(true);
                    w.set_fres(true);
                });

                Timer::after(Duration::from_millis(100)).await;

                regs.btable().write(|w| w.set_btable(0));
                regs.cntr().write(|w| {
                    w.set_pdwn(false);
                    w.set_fres(false);
                    w.set_resetm(true);
                    w.set_suspm(true);
                    w.set_wkupm(true);
                    w.set_ctrm(true);
                });

                #[cfg(usb_v2)]
                regs.bcdr().write(|w| w.set_dppu(true))
            }

            trace!("enabled");

            Bus {
                phantom: PhantomData,
            }
        }
    }
}

pub struct Bus<'d, T: Instance> {
    phantom: PhantomData<&'d mut T>,
}

impl<'d, T: Instance> driver::Bus for Bus<'d, T> {
    type PollFuture<'a> = impl Future<Output = Event> + 'a where Self: 'a;

    fn poll<'a>(&'a mut self) -> Self::PollFuture<'a> {
        poll_fn(|cx| unsafe {
            BUS_WAKER.register(cx.waker());
            let regs = T::regs();

            let istr = regs::Istr(IRQ_FLAGS.load(Ordering::Acquire));

            if istr.wkup() {
                let mut mask = regs::Istr(!0);
                mask.set_wkup(false);
                IRQ_FLAGS.fetch_and(mask.0, Ordering::AcqRel);

                // Required by datasheet
                regs.cntr().modify(|w| w.set_fsusp(false));
                return Poll::Ready(Event::Resume);
            }
            if istr.reset() {
                let mut mask = regs::Istr(!0);
                mask.set_reset(false);
                IRQ_FLAGS.fetch_and(mask.0, Ordering::AcqRel);
                return Poll::Ready(Event::Reset);
            }
            if istr.susp() {
                let mut mask = regs::Istr(!0);
                mask.set_susp(false);
                IRQ_FLAGS.fetch_and(mask.0, Ordering::AcqRel);
                return Poll::Ready(Event::Suspend);
            }

            Poll::Pending
        })
    }

    #[inline]
    fn reset(&mut self) {
        self.set_configured(false);
        let regs = T::regs();

        unsafe {
            trace!("RESET REGS WRITINGINGING");
            regs.daddr().write(|w| {
                w.set_ef(true);
                w.set_add(0);
            });

            regs.epr(0).write(|w| {
                w.set_ep_type(vals::EpType::CONTROL);
                w.set_stat_rx(vals::StatRx::VALID);
                w.set_stat_tx(vals::StatTx::NAK);
            });
        }
    }

    #[inline]
    fn set_configured(&mut self, configured: bool) {
        let regs = T::regs();

        // TODO

        for i in 1..=7 {
            In::waker(i).wake();
            Out::waker(i).wake();
        }
    }

    #[inline]
    fn set_device_address(&mut self, _addr: u8) {
        // Nothing to do, the peripheral handles this.
    }

    fn set_stalled(&mut self, ep_addr: EndpointAddress, stalled: bool) {
        Driver::<T>::set_stalled(ep_addr, stalled)
    }

    fn is_stalled(&mut self, ep_addr: EndpointAddress) -> bool {
        Driver::<T>::is_stalled(ep_addr)
    }

    #[inline]
    fn suspend(&mut self) {
        let regs = T::regs();
        // TODO
    }

    #[inline]
    fn resume(&mut self) {
        let regs = T::regs();
        // TODO
    }
}

pub enum Out {}
pub enum In {}

trait EndpointDir {
    fn waker(i: usize) -> &'static AtomicWaker;
    fn mask(i: usize) -> u32;
}

impl EndpointDir for In {
    #[inline]
    fn waker(i: usize) -> &'static AtomicWaker {
        &EP_IN_WAKERS[i - 1]
    }

    #[inline]
    fn mask(i: usize) -> u32 {
        1 << i
    }
}

impl EndpointDir for Out {
    #[inline]
    fn waker(i: usize) -> &'static AtomicWaker {
        &EP_OUT_WAKERS[i - 1]
    }

    #[inline]
    fn mask(i: usize) -> u32 {
        1 << (i + 16)
    }
}

pub struct Endpoint<'d, T: Instance, Dir> {
    _phantom: PhantomData<(&'d mut T, Dir)>,
    info: EndpointInfo,
    index: u8,
}

impl<'d, T: Instance, Dir: EndpointDir> driver::Endpoint for Endpoint<'d, T, Dir> {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }

    fn set_stalled(&self, stalled: bool) {
        Driver::<T>::set_stalled(self.info.addr, stalled)
    }

    fn is_stalled(&self) -> bool {
        Driver::<T>::is_stalled(self.info.addr)
    }

    type WaitEnabledFuture<'a> = impl Future<Output = ()> + 'a where Self: 'a;

    fn wait_enabled(&mut self) -> Self::WaitEnabledFuture<'_> {
        let i = self.info.addr.index();
        assert!(i != 0);

        poll_fn(move |cx| {
            Dir::waker(i).register(cx.waker());
            Poll::Pending

            /*
            // TODO
            if Dir::is_enabled(T::regs(), i) {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
                 */
        })
    }
}

impl<'d, T: Instance> driver::EndpointOut for Endpoint<'d, T, Out> {
    type ReadFuture<'a> = impl Future<Output = Result<usize, EndpointError>> + 'a where Self: 'a;

    fn read<'a>(&'a mut self, buf: &'a mut [u8]) -> Self::ReadFuture<'a> {
        async move {
            todo!();
        }
    }
}

impl<'d, T: Instance> driver::EndpointIn for Endpoint<'d, T, In> {
    type WriteFuture<'a> = impl Future<Output = Result<(), EndpointError>> + 'a where Self: 'a;

    fn write<'a>(&'a mut self, buf: &'a [u8]) -> Self::WriteFuture<'a> {
        async move {
            todo!();
        }
    }
}

pub struct ControlPipe<'d, T: Instance> {
    _phantom: PhantomData<&'d mut T>,
    max_packet_size: u16,
}

impl<'d, T: Instance> driver::ControlPipe for ControlPipe<'d, T> {
    type SetupFuture<'a> = impl Future<Output = Request> + 'a where Self: 'a;
    type DataOutFuture<'a> = impl Future<Output = Result<usize, EndpointError>> + 'a where Self: 'a;
    type DataInFuture<'a> = impl Future<Output = Result<(), EndpointError>> + 'a where Self: 'a;

    fn max_packet_size(&self) -> usize {
        usize::from(self.max_packet_size)
    }

    fn setup<'a>(&'a mut self) -> Self::SetupFuture<'a> {
        async move {
            loop {
                Timer::after(Duration::from_secs(1)).await;
            }
            // TODO

            let buf = [0; 8];
            let req = Request::parse(&buf);
            req
        }
    }

    fn data_out<'a>(&'a mut self, buf: &'a mut [u8]) -> Self::DataOutFuture<'a> {
        async move {
            let regs = T::regs();
            todo!();
        }
    }

    fn data_in<'a>(&'a mut self, buf: &'a [u8], last_packet: bool) -> Self::DataInFuture<'a> {
        async move {
            todo!();
        }
    }

    fn accept(&mut self) {
        let regs = T::regs();
        todo!();
    }

    fn reject(&mut self) {
        let regs = T::regs();
        todo!();
    }
}

fn dma_start() {
    compiler_fence(Ordering::Release);
}

fn dma_end() {
    compiler_fence(Ordering::Acquire);
}

pub(crate) mod sealed {
    pub trait Instance {
        fn regs() -> crate::pac::usb::Usb;
    }
}

pub trait Instance: sealed::Instance + RccPeripheral + 'static {
    type Interrupt: Interrupt;
}

// Internal PHY pins
pin_trait!(DpPin, Instance);
pin_trait!(DmPin, Instance);

foreach_interrupt!(
    ($inst:ident, usb, $block:ident, HP, $irq:ident) => {
        impl sealed::Instance for crate::peripherals::$inst {
            fn regs() -> crate::pac::usb::Usb {
                crate::pac::$inst
            }
        }

        impl Instance for crate::peripherals::$inst {
            type Interrupt = crate::interrupt::$irq;
        }
    };

);

#[cfg(any(stm32l0, stm32l1))]
const DP_PULL_UP_FEATURE: bool = true;
#[cfg(any(stm32f1, stm32f3, stm32l4, stm32l5))]
const DP_PULL_UP_FEATURE: bool = false;

const EP_MEMORY: *const () = 0x4000_6000 as _;

#[cfg(any(stm32f1, stm32l1, stm32f303xb, stm32f303xc))]
const EP_MEMORY_SIZE: usize = 512;
#[cfg(any(stm32l0, stm32l4, stm32l5, stm32f303xd, stm32f303xe))]
const EP_MEMORY_SIZE: usize = 1024;

#[cfg(any(stm32f1, stm32l1, stm32f303xb, stm32f303xc))]
const EP_MEMORY_ACCESS_2X16: bool = false;
#[cfg(any(stm32l0, stm32l4, stm32l5, stm32f303xd, stm32f303xe))]
const EP_MEMORY_ACCESS_2X16: bool = true;
