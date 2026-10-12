//! Application-owned external voltage domain policy for `SPC0.EVD_CFG`.
//!
//! The HAL does not choose this board-specific policy. `EVD_CFG` resets only
//! on POR, so applications should establish it on every boot.
//! VDD bit 0 is rejected: isolation there is reserved for Deep Power Down,
//! which this HAL does not support (MCXA2 Table 167, MCXA5 Table 395).

/// MCXA2 permits VDD_USB isolation; VDD and reserved bits are excluded.
#[cfg(feature = "mcxa2xx")]
const SELECTABLE_SUPPLIES: u8 = 1 << 1;

/// MCXA5 permits VDD_USB, VDD_P2/ANA, VDD_P3, VDD_P4 and VDD_VBAT isolation.
#[cfg(feature = "mcxa5xx")]
const SELECTABLE_SUPPLIES: u8 = 0b0011_1110;

/// External voltage domain policy errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum EvdPolicyError {
    /// A mask names VDD, a reserved bit or a supply absent on this family.
    UnsupportedSupply,
    /// The writable policy fields did not read back as written.
    ReadbackMismatch,
}

#[cfg(test)]
mod tests {
    use super::{EvdPolicyError, ExternalVoltageDomainPolicy};

    #[test]
    fn validates_both_masks_against_the_family_supply_mapping() {
        // Independent board-policy contract: VDD and reserved/absent supplies
        // must be rejected, even though some fit in the PAC register field.
        #[cfg(feature = "mcxa2xx")]
        const ALLOWED: u8 = 0b0000_0010;
        #[cfg(feature = "mcxa5xx")]
        const ALLOWED: u8 = 0b0011_1110;

        for always in 0..=u8::MAX {
            for low_power in 0..=u8::MAX {
                let result = ExternalVoltageDomainPolicy::try_new(always, low_power);
                if always & !ALLOWED == 0 && low_power & !ALLOWED == 0 {
                    let policy = result.unwrap();
                    assert_eq!(policy.always_isolated, always);
                    assert_eq!(policy.low_power_isolated, low_power);
                } else {
                    assert_eq!(result, Err(EvdPolicyError::UnsupportedSupply));
                }
            }
        }
        assert_eq!(
            ExternalVoltageDomainPolicy::all_domains_powered(),
            ExternalVoltageDomainPolicy::try_new(0, 0).unwrap()
        );
    }
}

/// A checked `SPC0.EVD_CFG` policy for this chip family's external supplies.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ExternalVoltageDomainPolicy {
    /// `EVDISO`: domains isolated at all times.
    always_isolated: u8,
    /// `EVDLPISO`: domains isolated only while `SPC_LPREQ` is asserted.
    low_power_isolated: u8,
}

impl ExternalVoltageDomainPolicy {
    /// No external voltage domain is isolated.
    pub const fn all_domains_powered() -> Self {
        Self {
            always_isolated: 0,
            low_power_isolated: 0,
        }
    }

    /// Build `EVDISO` and `EVDLPISO` masks using the family's supply mapping
    /// (MCXA2 Table 167, MCXA5 Table 395).
    ///
    /// Returns [`EvdPolicyError::UnsupportedSupply`] for VDD bit 0, reserved
    /// bits or bits beyond this family's supply count.
    pub const fn try_new(always_isolated_mask: u8, low_power_isolated_mask: u8) -> Result<Self, EvdPolicyError> {
        if (always_isolated_mask & !SELECTABLE_SUPPLIES) != 0 || (low_power_isolated_mask & !SELECTABLE_SUPPLIES) != 0 {
            return Err(EvdPolicyError::UnsupportedSupply);
        }
        Ok(Self {
            always_isolated: always_isolated_mask,
            low_power_isolated: low_power_isolated_mask,
        })
    }

    /// Replace `SPC0.EVD_CFG` policy and verify its writable fields.
    ///
    /// A full write discards stale policy surviving non-POR resets. Status
    /// and reserved bits are written as zero and excluded from readback.
    /// Returns [`EvdPolicyError::ReadbackMismatch`] if the policy did not take.
    /// The application calls this; HAL init and sleep paths do not.
    pub fn apply(&self) -> Result<(), EvdPolicyError> {
        let spc = crate::pac::SPC0;

        spc.evd_cfg().write(|w| {
            w.set_evdiso(self.always_isolated);
            w.set_evdlpiso(self.low_power_isolated);
        });

        let readback = spc.evd_cfg().read();
        if readback.evdiso() == self.always_isolated && readback.evdlpiso() == self.low_power_isolated {
            Ok(())
        } else {
            Err(EvdPolicyError::ReadbackMismatch)
        }
    }
}
