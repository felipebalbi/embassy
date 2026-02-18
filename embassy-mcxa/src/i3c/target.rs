//! I3C target driver

use core::marker::PhantomData;

use embassy_hal_internal::Peri;
use embassy_hal_internal::drop::OnDrop;
use embassy_hal_internal::interrupt::InterruptExt;

use super::{Async, Blocking, Info, Instance, InterruptHandler, Mode, SclPin, SdaPin};
use crate::clocks::periph_helpers::{Div4, I3cClockSel, I3cConfig};
use crate::clocks::{ClockError, PoweredClock, WakeGuard, enable_and_reset};
use crate::gpio::{AnyPin, SealedPin};
pub use crate::i2c::controller::Speed;
use crate::interrupt::typelevel;
use crate::pac::i3c::vals::{
    Disto, Hkeep, Ibiresp, MctrlDir as I3cDir, MdatactrlRxtrig, MdatactrlTxtrig, Mstena, Request, State, Type,
};

const MAX_CHUNK_SIZE: usize = 255;
