//! Type-safe spoke coordinate types for radar angle calculations.
//!
//! This module provides three distinct types to prevent confusion between:
//! - `SpokeAngle`: Angle relative to ship's bow (0 = directly ahead, clockwise)
//! - `SpokeBearing`: Bearing relative to True North (0 = North, clockwise)
//! - `SpokeHeading`: Ship's heading (bearing of the bow relative to True North)
//!
//! All values are in "spokes" (radar units in range `[0..spokes_per_revolution)`).
//!
//! Relationship: `bearing = angle + heading` (mod spokes)
//! Therefore: `heading = bearing - angle` (mod spokes)
//! And: `angle = bearing - heading` (mod spokes)

use std::f64::consts::PI;
use std::fmt;

/// Angle relative to ship's bow in spokes. 0 = directly ahead, clockwise.
#[derive(Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct SpokeAngle(u32);

/// Bearing relative to True North in spokes. 0 = North, clockwise.
#[derive(Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct SpokeBearing(u32);

/// Ship heading: the bearing of the ship's bow relative to True North.
/// This is distinct from SpokeBearing to prevent accidental mixing.
#[derive(Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct SpokeHeading(u32);

// ============================================================================
// SpokeAngle implementation
// ============================================================================

impl SpokeAngle {
    /// Create a new SpokeAngle, normalizing to [0..spokes)
    #[inline]
    pub fn new(value: i32, spokes: u32) -> Self {
        Self(value.rem_euclid(spokes as i32) as u32)
    }

    /// Create from a raw u32 value (must already be normalized)
    #[inline]
    pub fn from_raw(value: u32) -> Self {
        Self(value)
    }

    /// Get the raw value
    #[inline]
    pub fn raw(&self) -> u32 {
        self.0
    }

    /// Get as i32 for calculations
    #[inline]
    pub fn as_i32(&self) -> i32 {
        self.0 as i32
    }

    /// Re-normalize if spokes_per_revolution changed
    #[inline]
    pub fn normalize(&self, spokes: u32) -> Self {
        Self::new(self.0 as i32, spokes)
    }

    /// Add a signed offset, returning normalized result
    #[inline]
    pub fn add(&self, offset: i32, spokes: u32) -> Self {
        Self::new(self.0 as i32 + offset, spokes)
    }

    /// Subtract a signed offset, returning normalized result
    #[inline]
    pub fn sub(&self, offset: i32, spokes: u32) -> Self {
        Self::new(self.0 as i32 - offset, spokes)
    }

    /// Calculate signed difference to another angle (shortest path)
    pub fn diff(&self, other: &Self, spokes: u32) -> i32 {
        let diff = other.0 as i32 - self.0 as i32;
        let half = spokes as i32 / 2;
        if diff > half {
            diff - spokes as i32
        } else if diff < -half {
            diff + spokes as i32
        } else {
            diff
        }
    }

    /// Convert to radians [0, 2π)
    #[inline]
    pub fn to_radians(&self, spokes: u32) -> f64 {
        self.0 as f64 * 2.0 * PI / spokes as f64
    }

    /// Create from radians, normalizing to [0..spokes)
    #[inline]
    pub fn from_radians(radians: f64, spokes: u32) -> Self {
        let value = (radians * spokes as f64 / (2.0 * PI)).round() as i32;
        Self::new(value, spokes)
    }

    /// Convert to SpokeBearing by adding heading
    /// bearing = angle + heading
    #[inline]
    pub fn to_bearing(&self, heading: SpokeHeading, spokes: u32) -> SpokeBearing {
        SpokeBearing::new(self.0 as i32 + heading.0 as i32, spokes)
    }

    /// Check if this angle is between start and end (inclusive), handling wraparound
    pub fn is_between(&self, start: SpokeAngle, end: SpokeAngle, spokes: u32) -> bool {
        let angle = self.0.rem_euclid(spokes);
        let start = start.0.rem_euclid(spokes);
        let end = end.0.rem_euclid(spokes);

        if start <= end {
            angle >= start && angle <= end
        } else {
            // Wraps around 0
            angle >= start || angle <= end
        }
    }
}

impl fmt::Debug for SpokeAngle {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "SpokeAngle({})", self.0)
    }
}

impl fmt::Display for SpokeAngle {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}

// ============================================================================
// SpokeBearing implementation
// ============================================================================

impl SpokeBearing {
    /// Create a new SpokeBearing, normalizing to [0..spokes)
    #[inline]
    pub fn new(value: i32, spokes: u32) -> Self {
        Self(value.rem_euclid(spokes as i32) as u32)
    }

    /// Create from a raw u32 value (must already be normalized)
    #[inline]
    pub fn from_raw(value: u32) -> Self {
        Self(value)
    }

    /// Get the raw value
    #[inline]
    pub fn raw(&self) -> u32 {
        self.0
    }

    /// Get as i32 for calculations
    #[inline]
    pub fn as_i32(&self) -> i32 {
        self.0 as i32
    }

    /// Re-normalize if spokes_per_revolution changed
    #[inline]
    pub fn normalize(&self, spokes: u32) -> Self {
        Self::new(self.0 as i32, spokes)
    }

    /// Add a signed offset, returning normalized result
    #[inline]
    pub fn add(&self, offset: i32, spokes: u32) -> Self {
        Self::new(self.0 as i32 + offset, spokes)
    }

    /// Subtract a signed offset, returning normalized result
    #[inline]
    pub fn sub(&self, offset: i32, spokes: u32) -> Self {
        Self::new(self.0 as i32 - offset, spokes)
    }

    /// Calculate signed difference to another bearing (shortest path)
    pub fn diff(&self, other: &Self, spokes: u32) -> i32 {
        let diff = other.0 as i32 - self.0 as i32;
        let half = spokes as i32 / 2;
        if diff > half {
            diff - spokes as i32
        } else if diff < -half {
            diff + spokes as i32
        } else {
            diff
        }
    }

    /// Convert to radians [0, 2π)
    #[inline]
    pub fn to_radians(&self, spokes: u32) -> f64 {
        self.0 as f64 * 2.0 * PI / spokes as f64
    }

    /// Create from radians, normalizing to [0..spokes)
    #[inline]
    pub fn from_radians(radians: f64, spokes: u32) -> Self {
        let value = (radians * spokes as f64 / (2.0 * PI)).round() as i32;
        Self::new(value, spokes)
    }

    /// Convert to SpokeAngle by subtracting heading
    /// angle = bearing - heading
    #[inline]
    pub fn to_angle(&self, heading: SpokeHeading, spokes: u32) -> SpokeAngle {
        SpokeAngle::new(self.0 as i32 - heading.0 as i32, spokes)
    }

    /// Check if this bearing is between start and end (inclusive), handling wraparound
    pub fn is_between(&self, start: SpokeBearing, end: SpokeBearing, spokes: u32) -> bool {
        let bearing = self.0.rem_euclid(spokes);
        let start = start.0.rem_euclid(spokes);
        let end = end.0.rem_euclid(spokes);

        if start <= end {
            bearing >= start && bearing <= end
        } else {
            // Wraps around 0
            bearing >= start || bearing <= end
        }
    }
}

impl fmt::Debug for SpokeBearing {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "SpokeBearing({})", self.0)
    }
}

impl fmt::Display for SpokeBearing {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}

// ============================================================================
// SpokeHeading implementation
// ============================================================================

impl SpokeHeading {
    /// Create a new SpokeHeading, normalizing to [0..spokes)
    #[inline]
    pub fn new(value: i32, spokes: u32) -> Self {
        Self(value.rem_euclid(spokes as i32) as u32)
    }

    /// Create from a raw u32 value (must already be normalized)
    #[inline]
    pub fn from_raw(value: u32) -> Self {
        Self(value)
    }

    /// Create heading from bearing and angle
    /// heading = bearing - angle
    #[inline]
    pub fn from_bearing_and_angle(bearing: SpokeBearing, angle: SpokeAngle, spokes: u32) -> Self {
        Self::new(bearing.0 as i32 - angle.0 as i32, spokes)
    }

    /// Get the raw value
    #[inline]
    pub fn raw(&self) -> u32 {
        self.0
    }

    /// Get as i32 for calculations
    #[inline]
    pub fn as_i32(&self) -> i32 {
        self.0 as i32
    }

    /// Re-normalize if spokes_per_revolution changed
    #[inline]
    pub fn normalize(&self, spokes: u32) -> Self {
        Self::new(self.0 as i32, spokes)
    }

    /// Add a signed offset, returning normalized result
    #[inline]
    pub fn add(&self, offset: i32, spokes: u32) -> Self {
        Self::new(self.0 as i32 + offset, spokes)
    }

    /// Subtract a signed offset, returning normalized result
    #[inline]
    pub fn sub(&self, offset: i32, spokes: u32) -> Self {
        Self::new(self.0 as i32 - offset, spokes)
    }

    /// Convert to radians [0, 2π)
    #[inline]
    pub fn to_radians(&self, spokes: u32) -> f64 {
        self.0 as f64 * 2.0 * PI / spokes as f64
    }

    /// Create from radians, normalizing to [0..spokes)
    #[inline]
    pub fn from_radians(radians: f64, spokes: u32) -> Self {
        let value = (radians * spokes as f64 / (2.0 * PI)).round() as i32;
        Self::new(value, spokes)
    }

    /// Zero heading (ship pointing North)
    #[inline]
    pub fn zero() -> Self {
        Self(0)
    }
}

impl fmt::Debug for SpokeHeading {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "SpokeHeading({})", self.0)
    }
}

impl fmt::Display for SpokeHeading {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SPOKES: u32 = 2048;

    #[test]
    fn test_angle_normalization() {
        assert_eq!(SpokeAngle::new(0, SPOKES).raw(), 0);
        assert_eq!(SpokeAngle::new(100, SPOKES).raw(), 100);
        assert_eq!(SpokeAngle::new(-100, SPOKES).raw(), SPOKES - 100);
        assert_eq!(SpokeAngle::new(SPOKES as i32, SPOKES).raw(), 0);
        assert_eq!(SpokeAngle::new(SPOKES as i32 + 100, SPOKES).raw(), 100);
    }

    #[test]
    fn test_bearing_to_angle_conversion() {
        // If bearing is 100 and heading is 50, angle should be 50
        let bearing = SpokeBearing::new(100, SPOKES);
        let heading = SpokeHeading::new(50, SPOKES);
        let angle = bearing.to_angle(heading, SPOKES);
        assert_eq!(angle.raw(), 50);

        // If bearing is 50 and heading is 100, angle should wrap
        let bearing = SpokeBearing::new(50, SPOKES);
        let heading = SpokeHeading::new(100, SPOKES);
        let angle = bearing.to_angle(heading, SPOKES);
        assert_eq!(angle.raw(), SPOKES - 50);
    }

    #[test]
    fn test_angle_to_bearing_conversion() {
        // If angle is 50 and heading is 50, bearing should be 100
        let angle = SpokeAngle::new(50, SPOKES);
        let heading = SpokeHeading::new(50, SPOKES);
        let bearing = angle.to_bearing(heading, SPOKES);
        assert_eq!(bearing.raw(), 100);
    }

    #[test]
    fn test_heading_from_bearing_and_angle() {
        // If bearing is 100 and angle is 50, heading should be 50
        let bearing = SpokeBearing::new(100, SPOKES);
        let angle = SpokeAngle::new(50, SPOKES);
        let heading = SpokeHeading::from_bearing_and_angle(bearing, angle, SPOKES);
        assert_eq!(heading.raw(), 50);
    }

    #[test]
    fn test_diff() {
        let a = SpokeAngle::new(100, SPOKES);
        let b = SpokeAngle::new(150, SPOKES);
        assert_eq!(a.diff(&b, SPOKES), 50);
        assert_eq!(b.diff(&a, SPOKES), -50);

        // Wraparound
        let a = SpokeAngle::new(10, SPOKES);
        let b = SpokeAngle::new(SPOKES as i32 - 10, SPOKES);
        assert_eq!(a.diff(&b, SPOKES), -20);
        assert_eq!(b.diff(&a, SPOKES), 20);
    }

    #[test]
    fn test_is_between() {
        let spokes = SPOKES;

        // Simple case: start < end
        let angle = SpokeAngle::new(50, spokes);
        let start = SpokeAngle::new(40, spokes);
        let end = SpokeAngle::new(60, spokes);
        assert!(angle.is_between(start, end, spokes));

        let outside = SpokeAngle::new(30, spokes);
        assert!(!outside.is_between(start, end, spokes));

        // Wraparound case: start > end (zone crosses 0)
        let start = SpokeAngle::new(2000, spokes);
        let end = SpokeAngle::new(100, spokes);
        let in_zone1 = SpokeAngle::new(2020, spokes);
        let in_zone2 = SpokeAngle::new(50, spokes);
        let outside = SpokeAngle::new(500, spokes);
        assert!(in_zone1.is_between(start, end, spokes));
        assert!(in_zone2.is_between(start, end, spokes));
        assert!(!outside.is_between(start, end, spokes));
    }

    #[test]
    fn test_to_radians() {
        let angle = SpokeAngle::new(SPOKES as i32 / 4, SPOKES);
        let radians = angle.to_radians(SPOKES);
        assert!((radians - PI / 2.0).abs() < 0.001);
    }

    #[test]
    fn test_roundtrip_radians() {
        let original = SpokeAngle::new(500, SPOKES);
        let radians = original.to_radians(SPOKES);
        let restored = SpokeAngle::from_radians(radians, SPOKES);
        assert_eq!(original.raw(), restored.raw());
    }
}
