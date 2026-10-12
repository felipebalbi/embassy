//! Wake-Up Unit pin and module wake sources.
//!
//! External inputs have typed detection modes and power scopes. Interrupt-mode
//! waits retain events across cancellation; Power Down recovery replays live arms.
//! PF/PMC use global input indices, while PE/PDC use bank-local indices.
//! The WUU has no MRCC gate and does not take a clock power vote.

use core::cell::Cell;
use core::marker::PhantomData;

use critical_section::CriticalSection;
use embassy_hal_internal::{Peri, PeripheralType};
use maitake_sync::WaitQueue;

use crate::clocks::config::CoreSleep;
use crate::gpio::{Flex, Pull};
use crate::interrupt::typelevel::{Binding, Handler, Interrupt as _};
use crate::pac::wuu::{De, Me, Pf, Wupdc, Wupe, Wupmc};

/// Which `PE`/`PDC` register bank a WUU pin lives in.
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub(crate) enum WuuPinRegisterBank {
    /// `PE1` / `PDC1`: global inputs 0..=15.
    Pe1Pdc1,
    /// `PE2` / `PDC2`: global inputs 16..=31.
    Pe2Pdc2,
}

/// Global PF/PMC index; distinct from the bank-local PE/PDC index.
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) struct GlobalWuuPinIndex(u8);

/// Bank-local PE/PDC index, restarting at zero in each bank.
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) struct BankedWuuPinIndex {
    bank: WuuPinRegisterBank,
    local_index: u8,
}

/// Register indices for one wake-capable pin.
#[derive(Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) struct WuuPinInfo {
    global_index: u8,
}

impl WuuPinInfo {
    /// Build a descriptor, rejecting indices outside the 32-input register file.
    pub(crate) const fn new(global_index: u8) -> Self {
        assert!(global_index < 32, "WUU pin index must be below 32");
        Self { global_index }
    }

    /// Address in the globally indexed `PF`/`PMC` arrays.
    pub(crate) const fn global_index(self) -> GlobalWuuPinIndex {
        GlobalWuuPinIndex(self.global_index)
    }

    /// Address in the bank-local `PE`/`PDC` registers.
    pub(crate) const fn banked_index(self) -> BankedWuuPinIndex {
        if self.global_index < 16 {
            BankedWuuPinIndex {
                bank: WuuPinRegisterBank::Pe1Pdc1,
                local_index: self.global_index,
            }
        } else {
            BankedWuuPinIndex {
                bank: WuuPinRegisterBank::Pe2Pdc2,
                local_index: self.global_index - 16,
            }
        }
    }
}

pub(crate) mod sealed {
    use embassy_hal_internal::PeripheralType;

    use super::WuuPinInfo;
    use crate::gpio::GpioPin;

    /// Sealed pin descriptor supplied by generated PAC metadata.
    pub(crate) trait SealedWuuPin: GpioPin + PeripheralType {
        /// The pin's WUU input descriptor.
        const INFO: WuuPinInfo;
    }

    /// Sealed external-pin detection scope and PMC encoding.
    pub trait SealedDetectionScope {
        /// Raw `WUPMCn` encoding selected by this scope.
        const PMC_CODE: u8;
        /// Whether detection remains active outside low-leakage modes.
        const ALL_POWER: bool;
    }

    /// Sealed capability for supported PE, PF, PDC and PMC facilities.
    pub trait SealedWuuArmablePin {}

    /// Sealed marker for the single WUU peripheral instance.
    pub trait SealedWuuInstance {}

    /// Sealed type-level wake mode.
    pub trait SealedWakeMode {
        /// Only interrupt mode can raise PF and be awaited.
        const IS_INTERRUPT: bool;
    }
}

/// Marker for a GPIO pin that is wired to a WUU input on the selected chip.
#[allow(private_bounds)]
pub trait WuuPin: sealed::SealedWuuPin {}

#[allow(private_bounds)]
impl<T: sealed::SealedWuuPin> WuuPin for T {}

#[doc(hidden)]
#[macro_export]
macro_rules! impl_wuu_pin {
    ($pin:ident, $global_index:literal) => {
        impl $crate::wuu::sealed::SealedWuuPin for $crate::peripherals::$pin {
            const INFO: $crate::wuu::WuuPinInfo = $crate::wuu::WuuPinInfo::new($global_index);
        }
    };
}

/// Gate index for `clk_16k[0]`, the LPO used by WUU detection.
const FROCLKE_CLK_16K_0: usize = 0;

/// Supported PF W1C flags on MCXA2xx.
#[cfg(feature = "mcxa2xx")]
pub(crate) const FAMILY_PF_WRITABLE_MASK: u32 = 0x3fff_ffff;
/// Supported PF W1C flags on MCXA5xx.
#[cfg(feature = "mcxa5xx")]
pub(crate) const FAMILY_PF_WRITABLE_MASK: u32 = 0x4fff_ffff;

/// Number of globally indexed WUU inputs.
const WUU_SLOTS: usize = 32;

// Waiters unlink on drop; these queues are never closed or reinitialized.
static WUU_WAIT_QUEUES: [WaitQueue; WUU_SLOTS] = [const { WaitQueue::new() }; WUU_SLOTS];
/// Retained per-input events, independent of waiter cancellation.
static WUU_EVENTS: critical_section::Mutex<Cell<u32>> = critical_section::Mutex::new(Cell::new(0));
/// Inputs currently armed in interrupt mode by a live handle.
static WUU_ARMED: critical_section::Mutex<Cell<u32>> = critical_section::Mutex::new(Cell::new(0));
/// Sticky diagnostic mask of PF observations not owned by a live handle.
static WUU_UNEXPECTED: critical_section::Mutex<Cell<u32>> = critical_section::Mutex::new(Cell::new(0));
/// Sticky mask of inputs whose arms could not be restored after Power Down.
static WUU_RECOVERY_FAILED: critical_section::Mutex<Cell<u32>> = critical_section::Mutex::new(Cell::new(0));
/// Desired module wake enables, retained for recovery.
static WUU_ME_DESIRED: critical_section::Mutex<Cell<u32>> = critical_section::Mutex::new(Cell::new(0));
/// Desired module DMA/trigger enables, retained for recovery.
static WUU_DE_DESIRED: critical_section::Mutex<Cell<u32>> = critical_section::Mutex::new(Cell::new(0));

/// What one WUU external input was last successfully armed with.
#[derive(Clone, Copy)]
struct WuuArmRecord {
    /// A successfully read-back arm exists for this input.
    live: bool,
    /// A replay of this record failed; the handle is no longer armed.
    invalid: bool,
    /// The recorded mode is interrupt, the only CPU-resuming one.
    interrupt: bool,
    /// Raw `WUPEn` encoding.
    pe_code: u8,
    /// Raw `WUPDCn` encoding.
    pdc_code: u8,
    /// Raw `WUPMCn` encoding.
    scope_code: u8,
}

impl WuuArmRecord {
    const fn empty() -> Self {
        Self {
            live: false,
            invalid: false,
            interrupt: false,
            pe_code: 0,
            pdc_code: 0,
            scope_code: 0,
        }
    }
}

/// One recovery record per globally indexed input.
static WUU_ARM_RECORDS: [critical_section::Mutex<Cell<WuuArmRecord>>; WUU_SLOTS] =
    [const { critical_section::Mutex::new(Cell::new(WuuArmRecord::empty())) }; WUU_SLOTS];

fn write_pmc(index: GlobalWuuPinIndex, scope: Wupmc) {
    crate::pac::WUU0.pmc().modify(|w| w.set_wupmc(index.0 as usize, scope));
}

fn clear_pf(index: GlobalWuuPinIndex) {
    // PF is W1C: write only the selected flag, never read-modify-write.
    crate::pac::WUU0.pf().write(|w| w.set_wuf(index.0 as usize, true));
}

fn write_pdc(index: BankedWuuPinIndex, mode: Wupdc) {
    let wuu = crate::pac::WUU0;
    match index.bank {
        WuuPinRegisterBank::Pe1Pdc1 => wuu.pdc1().modify(|w| w.set_wupdc(index.local_index as usize, mode)),
        WuuPinRegisterBank::Pe2Pdc2 => wuu.pdc2().modify(|w| w.set_wupdc(index.local_index as usize, mode)),
    }
}

fn write_pe(index: BankedWuuPinIndex, detection: Wupe) {
    let wuu = crate::pac::WUU0;
    match index.bank {
        WuuPinRegisterBank::Pe1Pdc1 => wuu.pe1().modify(|w| w.set_wupe(index.local_index as usize, detection)),
        WuuPinRegisterBank::Pe2Pdc2 => wuu.pe2().modify(|w| w.set_wupe(index.local_index as usize, detection)),
    }
}

fn read_pe(index: BankedWuuPinIndex) -> u8 {
    let wuu = crate::pac::WUU0;
    match index.bank {
        WuuPinRegisterBank::Pe1Pdc1 => wuu.pe1().read().wupe(index.local_index as usize).to_bits(),
        WuuPinRegisterBank::Pe2Pdc2 => wuu.pe2().read().wupe(index.local_index as usize).to_bits(),
    }
}

fn read_pdc(index: BankedWuuPinIndex) -> u8 {
    let wuu = crate::pac::WUU0;
    match index.bank {
        WuuPinRegisterBank::Pe1Pdc1 => wuu.pdc1().read().wupdc(index.local_index as usize).to_bits(),
        WuuPinRegisterBank::Pe2Pdc2 => wuu.pdc2().read().wupdc(index.local_index as usize).to_bits(),
    }
}

fn read_pmc(index: GlobalWuuPinIndex) -> u8 {
    crate::pac::WUU0.pmc().read().wupmc(index.0 as usize).to_bits()
}

fn arm_pin_registers(pin: WuuPinInfo, pdc_code: u8, pe_code: u8, scope_code: u8, clear_pending: bool) {
    let global = pin.global_index();
    let banked = pin.banked_index();
    write_pmc(global, Wupmc::LowPwrOnly);
    if clear_pending {
        clear_pf(global);
    }
    write_pdc(banked, Wupdc::from_bits(pdc_code));
    write_pe(banked, Wupe::from_bits(pe_code));
    // PE/PDC must not change while all-power detection is enabled.
    if scope_code != 0 {
        write_pmc(global, Wupmc::AnyPwr);
    }
}

fn verify_pin_registers(pin: WuuPinInfo, pdc_code: u8, pe_code: u8, scope_code: u8) -> bool {
    let pe = read_pe(pin.banked_index());
    let pdc = read_pdc(pin.banked_index());
    let pmc = read_pmc(pin.global_index());
    pe == pe_code && pdc == pdc_code && pmc == scope_code
}

fn disable_pin_registers(pin: WuuPinInfo, clear_pending: bool) {
    write_pmc(pin.global_index(), Wupmc::LowPwrOnly);
    write_pe(pin.banked_index(), Wupe::from_bits(0));
    write_pdc(pin.banked_index(), Wupdc::from_bits(0));
    if clear_pending {
        clear_pf(pin.global_index());
    }
}

// Drain PF before any indexed access or queue work can panic.
#[inline(always)]
fn wuu_pf_drain_raw() -> u32 {
    let snapshot = crate::pac::WUU0.pf().read().0;
    crate::pac::WUU0
        .pf()
        .write_value(Pf(snapshot & FAMILY_PF_WRITABLE_MASK));
    snapshot
}

/// A WUU pin with supported PE, PF, PDC and PMC facilities.
/// MCXA2xx P3_27 is not armable because input 30 lacks these facilities.
pub trait WuuArmablePin: WuuPin + sealed::SealedWuuArmablePin {}

impl<T: WuuPin + sealed::SealedWuuArmablePin> WuuArmablePin for T {}

/// A type-level WUU pin wake mode.
#[allow(private_bounds)]
pub trait WakeMode: sealed::SealedWakeMode {}

/// Wake the CPU through the WUU0 interrupt. The only waitable mode.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Interrupt;

/// Raise a DMA request. No WUU interrupt is guaranteed: the request may be
/// cleared on transfer completion, so this mode can never be awaited.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Dma;

/// Raise a trigger event. PF is never set in this mode, so it cannot be awaited.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Trigger;

impl sealed::SealedWakeMode for Interrupt {
    const IS_INTERRUPT: bool = true;
}
impl WakeMode for Interrupt {}
impl sealed::SealedWakeMode for Dma {
    const IS_INTERRUPT: bool = false;
}
impl WakeMode for Dma {}
impl sealed::SealedWakeMode for Trigger {
    const IS_INTERRUPT: bool = false;
}
impl WakeMode for Trigger {}

/// Power modes in which external-pin detection remains active.
#[allow(private_bounds)]
pub trait DetectionScope: sealed::SealedDetectionScope {}

/// Detect only in low-leakage modes (`Wupmc::LowPwrOnly`).
/// Compatible only with [`CoreSleep::PowerDown`]; other modes reject this scope.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PowerDownOnly;

/// Detect in every power mode (`Wupmc::AnyPwr`).
/// Required for Deep Sleep, which is not a low-leakage mode.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AllPowerModes;

impl sealed::SealedDetectionScope for PowerDownOnly {
    const PMC_CODE: u8 = 0;
    const ALL_POWER: bool = false;
}
impl DetectionScope for PowerDownOnly {}

impl sealed::SealedDetectionScope for AllPowerModes {
    const PMC_CODE: u8 = 1;
    const ALL_POWER: bool = true;
}
impl DetectionScope for AllPowerModes {}

/// Edge detection for the interrupt and DMA modes.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Edge {
    /// Detect a rising edge.
    Rising,
    /// Detect a falling edge.
    Falling,
    /// Detect either edge.
    Any,
}

/// Level detection, the only thing the trigger mode accepts.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum LevelSense {
    /// Detect a high level.
    High,
    /// Detect a low level.
    Low,
}

const fn edge_code(edge: Edge) -> u8 {
    match edge {
        Edge::Rising => 1,
        Edge::Falling => 2,
        Edge::Any => 3,
    }
}

const fn level_code(sense: LevelSense) -> u8 {
    match sense {
        LevelSense::High => 1,
        LevelSense::Low => 2,
    }
}

/// Validated pin detection, wake mode and explicit power scope.
/// Reserved PDC encodings and any-edge triggers are not representable.
#[derive(Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PinWake<M: WakeMode, S: DetectionScope> {
    pe_code: u8,
    pdc_code: u8,
    _mode: PhantomData<M>,
    _scope: PhantomData<S>,
}

impl<S: DetectionScope> PinWake<Interrupt, S> {
    /// Wake through the WUU0 interrupt on `edge`, detected in scope `S`.
    pub const fn interrupt(edge: Edge) -> Self {
        Self {
            pe_code: edge_code(edge),
            pdc_code: 0,
            _mode: PhantomData,
            _scope: PhantomData,
        }
    }
}

impl<S: DetectionScope> PinWake<Dma, S> {
    /// Raise a DMA request on `edge`, detected in scope `S`.
    pub const fn dma(edge: Edge) -> Self {
        Self {
            pe_code: edge_code(edge),
            pdc_code: 1,
            _mode: PhantomData,
            _scope: PhantomData,
        }
    }
}

impl<S: DetectionScope> PinWake<Trigger, S> {
    /// Raise a trigger event while the pin is at `sense`, detected in scope `S`.
    pub const fn level(sense: LevelSense) -> Self {
        Self {
            pe_code: level_code(sense),
            pdc_code: 2,
            _mode: PhantomData,
            _scope: PhantomData,
        }
    }
}

/// A module interrupt wake source for low-leakage modes.
/// The caller binds, enables and acknowledges the module's own interrupt;
/// module wakes do not dispatch WUU0 or complete a WUU wait.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ModuleWake {
    /// SPC interrupt.
    SpcInterrupt,
    /// RTC 1Hz interrupt or RTC interrupt, a single combined source.
    #[cfg(feature = "mcxa2xx")]
    RtcOneHzOrInterrupt,
    /// RTC interrupt.
    #[cfg(feature = "mcxa5xx")]
    RtcInterrupt,
    /// Wake Timer interrupt.
    WakeTimerInterrupt,
    /// TDET interrupt.
    #[cfg(feature = "mcxa2xx")]
    TdetInterrupt,
    /// TDET or VBAT interrupt; the caller acknowledges both status flags.
    #[cfg(feature = "mcxa5xx")]
    TdetOrVbatInterrupt,
    /// GPIO5 interrupt 0.
    #[cfg(feature = "mcxa5xx")]
    Gpio5Interrupt0,
    /// GPIO5 interrupt 1.
    #[cfg(feature = "mcxa5xx")]
    Gpio5Interrupt1,
    /// LPTMR0 interrupt.
    Lptmr0Interrupt,
    /// CMP0 interrupt.
    Cmp0Interrupt,
}

/// Transient DMA/trigger servicing while the CPU remains asleep.
/// This does not generate a CPU wake or run an interrupt handler.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ModuleDmaService {
    /// GPIO5 DMA request 0.
    #[cfg(feature = "mcxa5xx")]
    Gpio5Dma0,
    /// GPIO5 DMA request 1.
    #[cfg(feature = "mcxa5xx")]
    Gpio5Dma1,
    /// GPIO5 trigger 0.
    #[cfg(feature = "mcxa5xx")]
    Gpio5Trigger0,
    /// GPIO5 trigger 1.
    #[cfg(feature = "mcxa5xx")]
    Gpio5Trigger1,
    /// LPTMR0 DMA request.
    Lptmr0Dma,
    /// LPTMR0 trigger.
    Lptmr0Trigger,
    /// CMP0 DMA request.
    Cmp0Dma,
}

/// Sticky PF observations not owned by a live interrupt-mode handle.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct UnexpectedWakeFlags(u32);

impl UnexpectedWakeFlags {
    /// The raw PF bit positions observed.
    pub const fn bits(self) -> u32 {
        self.0
    }

    /// Whether nothing unexpected has been observed.
    pub const fn is_empty(self) -> bool {
        self.0 == 0
    }

    /// Build a mask to hand to `clear_unexpected_wake_flags`.
    pub const fn from_bits(bits: u32) -> Self {
        Self(bits)
    }
}

/// Why [`Wuu::new`] refused the peripheral without changing clock configuration.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum WuuCreateError {
    /// `clocks::init` has not run, so no initialized clock state exists.
    ClocksUnavailable,
    /// The initialized clocks do not provide the WUU LPO, `clk_16k[0]`.
    LpoClockUnavailable,
    /// Hardware FRO_EN or the CLKE0 output gate is off.
    LpoHardwareUnavailable,
}

/// A refused [`Wuu::new`], carrying back both consumed constructor resources.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct WuuCreateFailure<'d, T: Instance, B> {
    /// Why construction was refused.
    pub error: WuuCreateError,
    /// The original WUU0 peripheral, unmodified and with its NVIC line disabled.
    pub peripheral: Peri<'d, T>,
    /// The original interrupt binding.
    pub binding: B,
}

/// Why [`Wuu::arm_pin`] refused to arm, before writing or after rolling back.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum WuuArmError {
    /// Initialized clocks are unavailable for checking scope compatibility.
    ClocksUnavailable,
    /// The detection scope cannot wake the initialized `CoreSleep`.
    IncompatibleScope,
    /// Interrupt mode was requested while the WUU0 NVIC line is disabled.
    InterruptDisabled,
    /// Interrupt wake in Deep Sleep or Power Down requires `SCR.SEVONPEND`.
    SevOnPendClear,
    /// PE, PDC or PMC read-back mismatched; full teardown has already run.
    ReadbackMismatch,
}

/// An owned wake-capable pin with its concrete GPIO type erased.
/// Obtained by disarming an input or recovering an arming failure.
pub struct AnyWuuPin<'p> {
    flex: Flex<'p>,
    info: WuuPinInfo,
}

impl<'p> AnyWuuPin<'p> {
    /// Transfer the disabled pin into a flexible GPIO driver.
    pub fn into_flex(self) -> Flex<'p> {
        self.flex
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for AnyWuuPin<'_> {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "AnyWuuPin {{ info: {} }}", self.info);
    }
}

/// A refused pin arm, carrying back a disabled, reusable pin resource.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct WuuArmFailure<'p> {
    /// Why the arm was refused.
    pub error: WuuArmError,
    /// The disabled pin, so the caller can correct the request and retry.
    pub pin: AnyWuuPin<'p>,
}

/// Why masked Power Down recovery could not restore the WUU.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum WuuRecoveryError {
    /// An arm read-back differed from its record; the transaction was rolled back.
    ReplayMismatch {
        /// Global WUU input, 0..=31, whose replay was refused by hardware.
        global_input: u8,
    },
}

/// Is the WUU0 NVIC line enabled?
fn wuu_irq_enabled() -> bool {
    crate::interrupt::typelevel::WUU0::is_enabled()
}

/// Read SCB `SCR.SEVONPEND`.
fn sevonpend_set() -> bool {
    // SAFETY: SCB is a ZST; only SCR is read, without claiming core peripherals.
    let scb: cortex_m::peripheral::SCB = unsafe { core::mem::transmute(()) };
    (scb.scr.read() & (1 << 4)) != 0
}

/// Whether the scope detects in the selected sleep mode.
fn scope_wakes(sleep: CoreSleep, all_power: bool) -> bool {
    match sleep {
        CoreSleep::WfeUngated => all_power,
        CoreSleep::WfeGated => all_power,
        CoreSleep::DeepSleep => all_power,
        CoreSleep::PowerDown => true,
    }
}

/// Whether interrupt wake depends on `SCR.SEVONPEND` in this sleep mode.
fn requires_sevonpend(sleep: CoreSleep) -> bool {
    match sleep {
        CoreSleep::WfeUngated => false,
        CoreSleep::WfeGated => false,
        CoreSleep::DeepSleep => true,
        CoreSleep::PowerDown => true,
    }
}

/// Disable supported inputs and modules, drain flags, and forget software state.
fn reset_all_wuu_state(cs: CriticalSection) {
    crate::pac::WUU0.me().write_value(Me(0));
    crate::pac::WUU0.de().write_value(De(0));
    let mut i = 0;
    while i < WUU_SLOTS {
        if (FAMILY_PF_WRITABLE_MASK & (1u32 << i)) != 0 {
            disable_pin_registers(WuuPinInfo::new(i as u8), true);
        }
        WUU_ARM_RECORDS[i].borrow(cs).set(WuuArmRecord::empty());
        i += 1;
    }
    WUU_ARMED.borrow(cs).set(0);
    WUU_EVENTS.borrow(cs).set(0);
    WUU_UNEXPECTED.borrow(cs).set(0);
    WUU_RECOVERY_FAILED.borrow(cs).set(0);
    WUU_ME_DESIRED.borrow(cs).set(0);
    WUU_DE_DESIRED.borrow(cs).set(0);
}

/// The WUU peripheral instance.
#[allow(private_bounds)]
pub trait Instance: sealed::SealedWuuInstance + PeripheralType {
    /// Interrupt for this instance.
    type Interrupt: crate::interrupt::typelevel::Interrupt;
}

impl sealed::SealedWuuInstance for crate::peripherals::WUU0 {}

impl Instance for crate::peripherals::WUU0 {
    type Interrupt = crate::interrupt::typelevel::WUU0;
}

/// WUU0 interrupt handler.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let snapshot = wuu_pf_drain_raw();

        let owned = critical_section::with(|cs| {
            let armed = WUU_ARMED.borrow(cs).get();
            let owned = snapshot & armed;
            let events = WUU_EVENTS.borrow(cs);
            events.set(events.get() | owned);
            let unexpected = WUU_UNEXPECTED.borrow(cs);
            unexpected.set(unexpected.get() | (snapshot & !armed));
            owned
        });
        let mut i = 0;
        while i < WUU_SLOTS {
            if (owned & (1u32 << i)) != 0 {
                WUU_WAIT_QUEUES[i].wake_all();
            }
            i += 1;
        }
    }
}

/// The Wake-Up Unit driver.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Wuu<'d> {
    _lifetime: PhantomData<&'d mut ()>,
}

impl<'d> Wuu<'d> {
    /// Take WUU0, returning both resources on failure.
    /// Checks cached and hardware LPO availability without changing clocks.
    /// No MRCC gate or power vote is needed; stale state is cleared before enabling IRQs.
    pub fn new<T, B>(peri: Peri<'d, T>, irq: B) -> Result<Self, WuuCreateFailure<'d, T, B>>
    where
        T: Instance,
        B: Binding<T::Interrupt, InterruptHandler<T>> + 'd,
    {
        T::Interrupt::disable();

        let cached = crate::clocks::with_clocks(|c| c.clk_16k_vsys.is_some());
        let error = match cached {
            None => Some(WuuCreateError::ClocksUnavailable),
            Some(false) => Some(WuuCreateError::LpoClockUnavailable),
            Some(true) => {
                let vbat = crate::pac::VBAT0;
                let running = vbat.froctla().read().fro_en();
                let ungated = vbat.froclke().read().clke(FROCLKE_CLK_16K_0);
                if running && ungated {
                    None
                } else {
                    Some(WuuCreateError::LpoHardwareUnavailable)
                }
            }
        };
        if let Some(error) = error {
            return Err(WuuCreateFailure {
                error,
                peripheral: peri,
                binding: irq,
            });
        }

        critical_section::with(reset_all_wuu_state);

        let _stale = wuu_pf_drain_raw();

        T::Interrupt::unpend();

        // Leave flags arriving after the drain visible to the handler.
        let _fresh = crate::pac::WUU0.pf().read().0 & FAMILY_PF_WRITABLE_MASK;

        // SAFETY: the binding installs the handler and stale state has been cleared.
        unsafe { T::Interrupt::enable() };

        Ok(Self { _lifetime: PhantomData })
    }

    /// Arm a supported pin with an explicit detection scope.
    /// Configures the GPIO input and pull before enabling the WUU source.
    /// Validation failures return a disabled pin resource.
    /// Reads back PE, PDC and PMC; a mismatch fully disarms and returns the pin.
    pub fn arm_pin<'w, 'p, P, M, S>(
        &'w self,
        pin: Peri<'p, P>,
        pull: Pull,
        wake: PinWake<M, S>,
    ) -> Result<WuuInput<'w, 'p, M>, WuuArmFailure<'p>>
    where
        P: WuuArmablePin,
        M: WakeMode,
        S: DetectionScope,
    {
        let info = <P as sealed::SealedWuuPin>::INFO;
        let pin = AnyWuuPin {
            flex: Flex::new(pin),
            info,
        };
        self.arm_any_pin(pin, pull, wake)
    }

    /// Rearm an erased wake-capable pin with a new pull, mode and scope.
    /// On failure, returns the disabled resource for retry or GPIO reuse.
    pub fn arm_any_pin<'w, 'p, M, S>(
        &'w self,
        pin: AnyWuuPin<'p>,
        pull: Pull,
        wake: PinWake<M, S>,
    ) -> Result<WuuInput<'w, 'p, M>, WuuArmFailure<'p>>
    where
        M: WakeMode,
        S: DetectionScope,
    {
        self.arm_pin_inner(pin, pull, wake)
    }

    fn arm_pin_inner<'w, 'p, M: WakeMode, S: DetectionScope>(
        &'w self,
        mut pin: AnyWuuPin<'p>,
        pull: Pull,
        wake: PinWake<M, S>,
    ) -> Result<WuuInput<'w, 'p, M>, WuuArmFailure<'p>> {
        pin.flex.set_as_disabled();
        let Some(sleep) = crate::clocks::with_clocks(|c| c.core_sleep) else {
            return Err(WuuArmFailure {
                error: WuuArmError::ClocksUnavailable,
                pin,
            });
        };

        if !scope_wakes(sleep, <S as sealed::SealedDetectionScope>::ALL_POWER) {
            return Err(WuuArmFailure {
                error: WuuArmError::IncompatibleScope,
                pin,
            });
        }

        if <M as sealed::SealedWakeMode>::IS_INTERRUPT {
            if !wuu_irq_enabled() {
                return Err(WuuArmFailure {
                    error: WuuArmError::InterruptDisabled,
                    pin,
                });
            }
            if requires_sevonpend(sleep) && !sevonpend_set() {
                return Err(WuuArmFailure {
                    error: WuuArmError::SevOnPendClear,
                    pin,
                });
            }
        }

        let info = pin.info;
        let bit = 1u32 << info.global_index;
        let slot = info.global_index as usize;
        let scope_code = <S as sealed::SealedDetectionScope>::PMC_CODE;

        pin.flex.set_as_input();
        pin.flex.set_pull(pull);

        let verified = critical_section::with(|cs| {
            arm_pin_registers(info, wake.pdc_code, wake.pe_code, scope_code, true);
            let ok = verify_pin_registers(info, wake.pdc_code, wake.pe_code, scope_code);
            if !ok {
                disable_pin_registers(info, true);
                let armed = WUU_ARMED.borrow(cs);
                armed.set(armed.get() & !bit);
                let events = WUU_EVENTS.borrow(cs);
                events.set(events.get() & !bit);
                WUU_ARM_RECORDS[slot].borrow(cs).set(WuuArmRecord::empty());
                let sticky = WUU_RECOVERY_FAILED.borrow(cs);
                sticky.set(sticky.get() & !bit);
                return false;
            }
            let armed = WUU_ARMED.borrow(cs);
            if <M as sealed::SealedWakeMode>::IS_INTERRUPT {
                armed.set(armed.get() | bit);
            } else {
                armed.set(armed.get() & !bit);
            }
            let events = WUU_EVENTS.borrow(cs);
            events.set(events.get() & !bit);
            WUU_ARM_RECORDS[slot].borrow(cs).set(WuuArmRecord {
                live: true,
                invalid: false,
                interrupt: <M as sealed::SealedWakeMode>::IS_INTERRUPT,
                pe_code: wake.pe_code,
                pdc_code: wake.pdc_code,
                scope_code,
            });
            let sticky = WUU_RECOVERY_FAILED.borrow(cs);
            sticky.set(sticky.get() & !bit);
            true
        });

        if !verified {
            pin.flex.set_as_disabled();
            return Err(WuuArmFailure {
                error: WuuArmError::ReadbackMismatch,
                pin,
            });
        }

        Ok(WuuInput {
            _wuu: PhantomData,
            pin,
            torn_down: Cell::new(false),
            _mode: PhantomData,
        })
    }

    /// Enable or disable a module interrupt wake in low-leakage modes only.
    /// Idempotent; the source module's own interrupt dispatches, not WUU0.
    /// This cannot make a module wake Deep Sleep.
    pub fn power_down_module_wake(&self, source: ModuleWake, enable: bool) {
        critical_section::with(|cs| {
            crate::pac::WUU0.me().modify(|w| match source {
                ModuleWake::SpcInterrupt => w.set_wume0(enable),
                #[cfg(feature = "mcxa2xx")]
                ModuleWake::RtcOneHzOrInterrupt => w.set_wume1(enable),
                #[cfg(feature = "mcxa5xx")]
                ModuleWake::RtcInterrupt => w.set_wume1(enable),
                ModuleWake::WakeTimerInterrupt => w.set_wume2(enable),
                #[cfg(feature = "mcxa2xx")]
                ModuleWake::TdetInterrupt => w.set_wume3(enable),
                #[cfg(feature = "mcxa5xx")]
                ModuleWake::TdetOrVbatInterrupt => w.set_wume3(enable),
                #[cfg(feature = "mcxa5xx")]
                ModuleWake::Gpio5Interrupt0 => w.set_wume4(enable),
                #[cfg(feature = "mcxa5xx")]
                ModuleWake::Gpio5Interrupt1 => w.set_wume5(enable),
                ModuleWake::Lptmr0Interrupt => w.set_wume6(enable),
                ModuleWake::Cmp0Interrupt => w.set_wume8(enable),
            });
            WUU_ME_DESIRED.borrow(cs).set(crate::pac::WUU0.me().read().0);
        });
    }

    /// Enable or disable transient DMA/trigger servicing for one module.
    /// Idempotent; this is not a CPU wake.
    pub fn module_dma_servicing(&self, source: ModuleDmaService, enable: bool) {
        critical_section::with(|cs| {
            crate::pac::WUU0.de().modify(|w| match source {
                #[cfg(feature = "mcxa5xx")]
                ModuleDmaService::Gpio5Dma0 => w.set_wude0(enable),
                #[cfg(feature = "mcxa5xx")]
                ModuleDmaService::Gpio5Dma1 => w.set_wude1(enable),
                #[cfg(feature = "mcxa5xx")]
                ModuleDmaService::Gpio5Trigger0 => w.set_wude2(enable),
                #[cfg(feature = "mcxa5xx")]
                ModuleDmaService::Gpio5Trigger1 => w.set_wude3(enable),
                ModuleDmaService::Lptmr0Dma => w.set_wude4(enable),
                ModuleDmaService::Lptmr0Trigger => w.set_wude6(enable),
                ModuleDmaService::Cmp0Dma => w.set_wude8(enable),
            });
            WUU_DE_DESIRED.borrow(cs).set(crate::pac::WUU0.de().read().0);
        });
    }

    /// Snapshot the sticky unexpected-source diagnostic mask. Non-blocking.
    pub fn unexpected_wake_flags(&self) -> UnexpectedWakeFlags {
        critical_section::with(|cs| UnexpectedWakeFlags(WUU_UNEXPECTED.borrow(cs).get()))
    }

    /// Clear the given bits of the sticky diagnostic mask. Non-blocking.
    pub fn clear_unexpected_wake_flags(&self, mask: UnexpectedWakeFlags) {
        critical_section::with(|cs| {
            let unexpected = WUU_UNEXPECTED.borrow(cs);
            unexpected.set(unexpected.get() & !mask.0);
        });
    }

    /// Sticky mask of inputs whose arms could not be restored after Power Down.
    /// Teardown, a fresh successful arm or a new WUU clears the corresponding diagnostic.
    pub fn power_down_recovery_failures(&self) -> u32 {
        critical_section::with(|cs| WUU_RECOVERY_FAILED.borrow(cs).get())
    }
}

impl Drop for Wuu<'_> {
    /// Leave no WUU destination enabled and no record behind.
    fn drop(&mut self) {
        crate::interrupt::typelevel::WUU0::disable();
        critical_section::with(reset_all_wuu_state);
    }
}

/// Exclusive armed input with immutable mode, detection and power scope.
/// To change configuration, disarm and arm the returned pin again.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct WuuInput<'w, 'p, M: WakeMode> {
    _wuu: PhantomData<&'w ()>,
    pin: AnyWuuPin<'p>,
    torn_down: Cell<bool>,
    _mode: PhantomData<M>,
}

impl<'p, M: WakeMode> WuuInput<'_, 'p, M> {
    /// Disable this input and clear its flag. Repeating it is harmless.
    pub fn disable(&mut self) {
        self.teardown();
    }

    /// Disable the input and return its pin for a fresh arm.
    pub fn disarm(self) -> AnyWuuPin<'p> {
        self.teardown();
        let this = core::mem::ManuallyDrop::new(self);
        // SAFETY: `this` is never dropped and `pin` is never read again.
        unsafe { core::ptr::read(&this.pin) }
    }

    /// Clear this input's pending flag and retained software event.
    pub fn clear_pending(&mut self) {
        let bit = 1u32 << self.pin.info.global_index;
        critical_section::with(|cs| {
            clear_pf(self.pin.info.global_index());
            let events = WUU_EVENTS.borrow(cs);
            events.set(events.get() & !bit);
        });
    }

    /// Whether recovery left this input unarmed.
    /// Repair by disarming and arming the returned pin again.
    pub fn power_down_recovery_failed(&self) -> bool {
        let bit = 1u32 << self.pin.info.global_index;
        critical_section::with(|cs| (WUU_RECOVERY_FAILED.borrow(cs).get() & bit) != 0)
    }

    /// Shared teardown for disable, disarm and Drop, run at most once.
    fn teardown(&self) {
        if self.torn_down.get() {
            return;
        }
        self.torn_down.set(true);
        let bit = 1u32 << self.pin.info.global_index;
        let slot = self.pin.info.global_index as usize;
        critical_section::with(|cs| {
            disable_pin_registers(self.pin.info, true);
            let armed = WUU_ARMED.borrow(cs);
            armed.set(armed.get() & !bit);
            let events = WUU_EVENTS.borrow(cs);
            events.set(events.get() & !bit);
            WUU_ARM_RECORDS[slot].borrow(cs).set(WuuArmRecord::empty());
            let sticky = WUU_RECOVERY_FAILED.borrow(cs);
            sticky.set(sticky.get() & !bit);
        });
        self.pin.flex.set_as_disabled();
    }
}

impl<M: WakeMode> Drop for WuuInput<'_, '_, M> {
    fn drop(&mut self) {
        self.teardown();
    }
}

impl WuuInput<'_, '_, Interrupt> {
    /// Wait for an interrupt-mode wake event.
    /// Cancellation removes the waiter but retains any unconsumed event for the next wait.
    pub async fn wait_for_wake(&mut self) {
        let slot = self.pin.info.global_index as usize;
        let bit = 1u32 << self.pin.info.global_index;

        let queue = &WUU_WAIT_QUEUES[slot];
        let events = &WUU_EVENTS;

        queue
            .wait_for(|| {
                critical_section::with(|cs| {
                    let events = events.borrow(cs);
                    let value = events.get();
                    if value & bit == 0 {
                        return false;
                    }
                    events.set(value & !bit);
                    true
                })
            })
            .await
            .expect("WUU wait queues are never closed");
    }
}

/// Require a live interrupt-mode arm with matching hardware configuration.
/// Also checks IRQ, SEVONPEND and hardware LPO readiness; ME/DE do not qualify.
pub(crate) fn power_down_preflight(cs: &CriticalSection) -> bool {
    let cs = *cs;

    let nvic_ready = wuu_irq_enabled();
    if !nvic_ready || !sevonpend_set() {
        return false;
    }

    let vbat = crate::pac::VBAT0;
    if !vbat.froctla().read().fro_en() || !vbat.froclke().read().clke(FROCLKE_CLK_16K_0) {
        return false;
    }

    let failed = WUU_RECOVERY_FAILED.borrow(cs).get();
    let mut i = 0;
    while i < WUU_SLOTS {
        let record = WUU_ARM_RECORDS[i].borrow(cs).get();
        let usable = record.live && record.interrupt && !record.invalid && (failed & (1u32 << i)) == 0;
        if usable {
            let info = WuuPinInfo::new(i as u8);
            let b = info.banked_index();
            let g = info.global_index();
            let pe = read_pe(b) == record.pe_code;
            let pdc = read_pdc(b) == record.pdc_code;
            let pmc = read_pmc(g) == record.scope_code;
            if pe && pdc && pmc {
                return true;
            }
        }
        i += 1;
    }
    false
}

/// Restore recorded arms and ME/DE as an all-or-nothing transaction.
/// Drains and publishes flags before and after replay; replay never clears PF.
///
/// # Safety
/// The caller must hold `cs` throughout recovery and keep interrupts masked since wake.
pub(crate) unsafe fn recover_after_power_down(cs: &CriticalSection) -> Result<(), WuuRecoveryError> {
    let cs = *cs;
    let wuu = crate::pac::WUU0;

    let snapshot = wuu.pf().read().0;
    wuu.pf().write_value(Pf(snapshot & FAMILY_PF_WRITABLE_MASK));

    // Publish the drained flags before replay; preserve new PF flags until the late drain.
    let armed = WUU_ARMED.borrow(cs).get();
    let owned = snapshot & armed;
    let events = WUU_EVENTS.borrow(cs);
    events.set(events.get() | owned);
    let unexpected = WUU_UNEXPECTED.borrow(cs);
    unexpected.set(unexpected.get() | (snapshot & !armed));
    let mut i = 0;
    while i < WUU_SLOTS {
        if (owned & (1u32 << i)) != 0 {
            WUU_WAIT_QUEUES[i].wake_all();
        }
        i += 1;
    }

    let mut i = 0;
    while i < WUU_SLOTS {
        if WUU_ARM_RECORDS[i].borrow(cs).get().live {
            disable_pin_registers(WuuPinInfo::new(i as u8), false);
        }
        i += 1;
    }
    wuu.me().write_value(Me(0));
    wuu.de().write_value(De(0));

    let mut failed_input: Option<u8> = None;
    let mut i = 0;
    while i < WUU_SLOTS {
        let record = WUU_ARM_RECORDS[i].borrow(cs).get();
        if record.live {
            let info = WuuPinInfo::new(i as u8);
            arm_pin_registers(info, record.pdc_code, record.pe_code, record.scope_code, false);
            let ok = verify_pin_registers(info, record.pdc_code, record.pe_code, record.scope_code);
            if !ok && failed_input.is_none() {
                failed_input = Some(i as u8);
            }
        }
        i += 1;
    }

    if let Some(global_input) = failed_input {
        let mut affected = 0u32;
        let mut i = 0;
        while i < WUU_SLOTS {
            let cell = WUU_ARM_RECORDS[i].borrow(cs);
            let mut record = cell.get();
            if record.live {
                disable_pin_registers(WuuPinInfo::new(i as u8), false);
                affected |= 1u32 << i;
                record.invalid = true;
                cell.set(record);
            }
            i += 1;
        }
        wuu.me().write_value(Me(0));
        wuu.de().write_value(De(0));

        // Publish failure and a retained synthetic event before waking affected waiters.
        let sticky = WUU_RECOVERY_FAILED.borrow(cs);
        sticky.set(sticky.get() | affected);

        let blocked = affected & WUU_ARMED.borrow(cs).get();
        let events = WUU_EVENTS.borrow(cs);
        events.set(events.get() | blocked);
        let mut i = 0;
        while i < WUU_SLOTS {
            if (blocked & (1u32 << i)) != 0 {
                WUU_WAIT_QUEUES[i].wake_all();
            }
            i += 1;
        }
        // PF remains pending for the ISR after interrupts are unmasked.
        return Err(WuuRecoveryError::ReplayMismatch { global_input });
    }

    wuu.me().write_value(Me(WUU_ME_DESIRED.borrow(cs).get()));
    wuu.de().write_value(De(WUU_DE_DESIRED.borrow(cs).get()));

    let late = wuu.pf().read().0;
    wuu.pf().write_value(Pf(late & FAMILY_PF_WRITABLE_MASK));
    let armed = WUU_ARMED.borrow(cs).get();
    let owned = late & armed;
    let events = WUU_EVENTS.borrow(cs);
    events.set(events.get() | owned);
    let unexpected = WUU_UNEXPECTED.borrow(cs);
    unexpected.set(unexpected.get() | (late & !armed));
    let mut i = 0;
    while i < WUU_SLOTS {
        if (owned & (1u32 << i)) != 0 {
            WUU_WAIT_QUEUES[i].wake_all();
        }
        i += 1;
    }

    Ok(())
}
