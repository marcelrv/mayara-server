//! ARPA (Automatic Radar Plotting Aid) - automatic target detection via guard zones
//!
//! This module handles automatic target acquisition within configured guard zones.
//! Targets detected here are passed to the main target tracking system.
//!
//! ## Coordinate Systems
//!
//! This module uses two coordinate systems for angles:
//! - `SpokeAngle`: Angle relative to ship's bow (0 = ahead). Used for guard zone configuration.
//! - `SpokeBearing`: Bearing relative to True North (0 = North). Used for blob tracking.
//! - `SpokeHeading`: Ship's heading (bearing of bow relative to North). Used for conversion.
//!
//! Relationship: `bearing = angle + heading` (mod spokes)

use std::cmp::{max, min};

use super::spoke_coords::{SpokeAngle, SpokeBearing, SpokeHeading};
use super::{MAX_BLOB_PIXELS, MIN_BLOB_PIXELS, MIN_BLOB_RANGE};
use crate::radar::GeoPosition;

/// Guard zone configuration for automatic target detection.
/// Angles are stored relative to ship's bow (SpokeAngle), distances in pixels.
#[derive(Debug, Clone)]
pub(crate) struct DetectionGuardZone {
    /// Start angle relative to ship's bow (0 = ahead)
    pub(crate) start_angle: SpokeAngle,
    /// End angle relative to ship's bow (0 = ahead)
    pub(crate) end_angle: SpokeAngle,
    /// Inner distance in pixels
    pub(crate) inner_range: i32,
    /// Outer distance in pixels
    pub(crate) outer_range: i32,
    /// Whether this guard zone is enabled for detection
    pub(crate) enabled: bool,
    /// Last scan time for each bearing to avoid duplicate detections
    last_scan_time: Vec<u64>,
    // Original config values (for recalculation when pixels_per_meter changes)
    config_start_angle_rad: f64,
    config_end_angle_rad: f64,
    config_inner_range_m: f64,
    config_outer_range_m: f64,
    config_enabled: bool,
}

impl DetectionGuardZone {
    pub(crate) fn new(spokes_per_revolution: i32) -> Self {
        Self {
            start_angle: SpokeAngle::from_raw(0),
            end_angle: SpokeAngle::from_raw(0),
            inner_range: 0,
            outer_range: 0,
            enabled: false,
            last_scan_time: vec![0; spokes_per_revolution as usize],
            config_start_angle_rad: 0.0,
            config_end_angle_rad: 0.0,
            config_inner_range_m: 0.0,
            config_outer_range_m: 0.0,
            config_enabled: false,
        }
    }

    /// Update zone from config (angles in radians, distances in meters)
    pub(crate) fn update_from_config(
        &mut self,
        start_angle_rad: f64,
        end_angle_rad: f64,
        inner_range_m: f64,
        outer_range_m: f64,
        enabled: bool,
        spokes_per_revolution: i32,
        pixels_per_meter: f64,
    ) {
        // Store original config values for later recalculation
        self.config_start_angle_rad = start_angle_rad;
        self.config_end_angle_rad = end_angle_rad;
        self.config_inner_range_m = inner_range_m;
        self.config_outer_range_m = outer_range_m;
        self.config_enabled = enabled;

        self.recalculate(spokes_per_revolution, pixels_per_meter);
    }

    /// Recalculate pixel/spoke values from stored config when pixels_per_meter changes
    pub(crate) fn recalculate(&mut self, spokes_per_revolution: i32, pixels_per_meter: f64) {
        self.enabled = self.config_enabled && pixels_per_meter > 0.0;

        if !self.enabled {
            return;
        }

        let spokes = spokes_per_revolution as u32;

        // Convert angles from radians to SpokeAngle (relative to bow)
        self.start_angle = SpokeAngle::from_radians(self.config_start_angle_rad, spokes);
        self.end_angle = SpokeAngle::from_radians(self.config_end_angle_rad, spokes);

        // Convert distances from meters to pixels
        self.inner_range = (self.config_inner_range_m * pixels_per_meter).max(1.0) as i32;
        self.outer_range = (self.config_outer_range_m * pixels_per_meter) as i32;

        // Calculate implied radar range (assuming 1024 pixels per spoke)
        let implied_range_m = if pixels_per_meter > 0.0 {
            1024.0 / pixels_per_meter
        } else {
            0.0
        };

        log::info!(
            "GuardZone configured: angles={}..{} spokes (relative to bow), range={}..{} pixels (config {}..{}m, ppm={:.4}, radar_range={:.0}m)",
            self.start_angle,
            self.end_angle,
            self.inner_range,
            self.outer_range,
            self.config_inner_range_m,
            self.config_outer_range_m,
            pixels_per_meter,
            implied_range_m
        );
    }

    /// Check if this zone has config that needs recalculation
    pub(crate) fn has_pending_config(&self) -> bool {
        self.config_enabled && !self.enabled
    }

    /// Check if an angle (relative to bow) is within this guard zone
    fn contains_angle(&self, angle: SpokeAngle, spokes: u32) -> bool {
        if !self.enabled {
            return false;
        }
        angle.is_between(self.start_angle, self.end_angle, spokes)
    }

    /// Check if a range (in pixels) is within this guard zone
    fn contains_range(&self, range: i32) -> bool {
        let in_range = self.enabled && range >= self.inner_range && range <= self.outer_range;
        // Log when a range check fails near the outer boundary
        if self.enabled && !in_range && range > self.outer_range && range <= self.outer_range + 50 {
            log::trace!(
                "Range {} just outside outer boundary {} (inner={})",
                range,
                self.outer_range,
                self.inner_range
            );
        }
        in_range
    }

    /// Check if a position is within the guard zone
    /// bearing: the bearing in geographic coordinates (0 = North)
    /// heading: the ship's heading (to convert geographic bearing to relative angle)
    pub(crate) fn contains(
        &self,
        bearing: SpokeBearing,
        range: i32,
        spokes: u32,
        heading: SpokeHeading,
    ) -> bool {
        // Convert geographic bearing to relative angle by subtracting heading
        // Guard zone angles are stored relative to ship heading
        let relative_angle = bearing.to_angle(heading, spokes);
        self.contains_angle(relative_angle, spokes) && self.contains_range(range)
    }
}

/// A blob being incrementally built as spokes arrive.
/// Used for automatic target detection within guard zones.
/// Bearings are geographic (relative to True North).
#[derive(Debug, Clone)]
pub(crate) struct BlobInProgress {
    /// Range values present on the last spoke that contributed to this blob
    /// Used to check adjacency with the next spoke
    last_spoke_ranges: Vec<i32>,
    /// The bearing of the last spoke that contributed pixels (geographic)
    last_bearing: SpokeBearing,
    /// Bounding box (geographic bearings)
    pub(crate) min_bearing: SpokeBearing,
    pub(crate) max_bearing: SpokeBearing,
    pub(crate) min_r: i32,
    pub(crate) max_r: i32,
    /// Total pixel count
    pub(crate) pixel_count: usize,
    /// Time when first pixel was seen
    pub(crate) start_time: u64,
    /// Own ship position when blob started
    pub(crate) start_pos: GeoPosition,
    /// Heading when blob was first detected (for guard zone validation)
    pub(crate) start_heading: SpokeHeading,
}

impl BlobInProgress {
    pub(crate) fn new(bearing: SpokeBearing, r: i32, time: u64, pos: GeoPosition, heading: SpokeHeading) -> Self {
        Self {
            last_spoke_ranges: vec![r],
            last_bearing: bearing,
            min_bearing: bearing,
            max_bearing: bearing,
            min_r: r,
            max_r: r,
            pixel_count: 1,
            start_time: time,
            start_pos: pos,
            start_heading: heading,
        }
    }

    /// Add a pixel to this blob
    pub(crate) fn add_pixel(&mut self, bearing: SpokeBearing, r: i32) {
        self.max_bearing = bearing; // bearing always increases as we process spokes
        self.min_r = min(self.min_r, r);
        self.max_r = max(self.max_r, r);
        self.pixel_count += 1;
    }

    /// Start a new spoke - clear last_spoke_ranges and set last_bearing
    pub(crate) fn start_new_spoke(&mut self, bearing: SpokeBearing) {
        self.last_spoke_ranges.clear();
        self.last_bearing = bearing;
    }

    /// Check if a range value on the current spoke is adjacent to this blob
    /// (i.e., within 1 pixel of any range on the previous spoke)
    pub(crate) fn is_adjacent(&self, r: i32) -> bool {
        self.last_spoke_ranges
            .iter()
            .any(|&prev_r| (prev_r - r).abs() <= 1)
    }

    /// Get the last bearing that contributed to this blob
    pub(crate) fn last_bearing(&self) -> SpokeBearing {
        self.last_bearing
    }

    /// Add a range to the last spoke ranges
    pub(crate) fn push_last_spoke_range(&mut self, r: i32) {
        self.last_spoke_ranges.push(r);
    }

    /// Calculate center position in polar coordinates (raw bearing, r)
    pub(crate) fn center(&self, spokes: u32) -> (i32, i32) {
        // Calculate center bearing - handle wraparound
        let min_b = self.min_bearing.as_i32();
        let max_b = self.max_bearing.as_i32();
        let center_bearing = if max_b >= min_b {
            (min_b + max_b) / 2
        } else {
            // Wraparound case
            let sum = min_b + max_b + spokes as i32;
            (sum / 2) % spokes as i32
        };
        let center_r = (self.min_r + self.max_r) / 2;
        (center_bearing, center_r)
    }

    /// Check if blob meets minimum size requirements
    pub(crate) fn is_valid(&self) -> bool {
        self.pixel_count >= MIN_BLOB_PIXELS
            && self.pixel_count <= MAX_BLOB_PIXELS
            && self.min_r >= MIN_BLOB_RANGE
    }
}

/// ARPA detector - handles automatic target detection within guard zones
#[derive(Debug, Clone)]
pub(crate) struct ArpaDetector {
    /// Guard zones for target detection
    pub(crate) guard_zones: [DetectionGuardZone; 2],
    /// Current heading (updated each spoke, used for guard zone checks)
    pub(crate) current_heading: SpokeHeading,
    /// Blobs currently being built as spokes arrive
    blobs_in_progress: Vec<BlobInProgress>,
    /// Previous bearing for detecting spoke gaps (geographic)
    prev_bearing: SpokeBearing,
}

impl ArpaDetector {
    pub(crate) fn new(spokes_per_revolution: i32) -> Self {
        Self {
            guard_zones: [
                DetectionGuardZone::new(spokes_per_revolution),
                DetectionGuardZone::new(spokes_per_revolution),
            ],
            current_heading: SpokeHeading::zero(),
            blobs_in_progress: Vec::new(),
            prev_bearing: SpokeBearing::from_raw(0),
        }
    }

    /// Check if any guard zone is enabled
    pub(crate) fn has_active_guard_zone(&self) -> bool {
        self.guard_zones.iter().any(|z| z.enabled)
    }

    /// Check if a position is within any enabled guard zone
    pub(crate) fn is_in_guard_zone(
        &self,
        bearing: SpokeBearing,
        range: i32,
        heading: SpokeHeading,
        spokes: u32,
    ) -> bool {
        self.guard_zones
            .iter()
            .any(|z| z.contains(bearing, range, spokes, heading))
    }

    /// Get which guard zone (1 or 2) contains the position, or 0 if none
    pub(crate) fn get_containing_zone(
        &self,
        bearing: SpokeBearing,
        range: i32,
        heading: SpokeHeading,
        spokes: u32,
    ) -> u8 {
        if self.guard_zones[0].contains(bearing, range, spokes, heading) {
            1
        } else if self.guard_zones[1].contains(bearing, range, spokes, heading) {
            2
        } else {
            0
        }
    }

    /// Recalculate guard zones when pixels_per_meter changes
    pub(crate) fn recalculate_zones(&mut self, spokes_per_revolution: i32, pixels_per_meter: f64) {
        for zone in &mut self.guard_zones {
            if zone.has_pending_config() || zone.enabled {
                zone.recalculate(spokes_per_revolution, pixels_per_meter);
            }
        }
    }

    /// Process strong pixels from a spoke and update blobs in progress.
    /// Returns completed blobs that should be passed to target acquisition.
    ///
    /// Parameters:
    /// - `bearing`: Geographic bearing of the spoke (0 = North)
    /// - `heading`: Ship's heading (bearing of bow)
    /// - `strong_pixels`: Range values of strong pixels on this spoke
    /// - `time`: Time of spoke
    /// - `pos`: Ship position at time of spoke
    /// - `spokes`: Number of spokes per revolution
    pub(crate) fn process_blob_pixels(
        &mut self,
        bearing: SpokeBearing,
        heading: SpokeHeading,
        strong_pixels: &[i32],
        time: u64,
        pos: GeoPosition,
        spokes: u32,
    ) -> Vec<BlobInProgress> {
        let mut completed_blobs = Vec::new();

        // Check if any guard zone is enabled - automatic target acquisition ONLY happens
        // within enabled guard zones (matching C++ radar_pi behavior)
        let guard_zone_active = self.guard_zones[0].enabled || self.guard_zones[1].enabled;

        // Log guard zone state once per rotation (at bearing 0)
        if bearing.raw() == 0 {
            log::debug!(
                "Guard zone state: active={}, zone0_enabled={}, zone0 angles {}..{} range {}..{}, zone1_enabled={}",
                guard_zone_active,
                self.guard_zones[0].enabled,
                self.guard_zones[0].start_angle,
                self.guard_zones[0].end_angle,
                self.guard_zones[0].inner_range,
                self.guard_zones[0].outer_range,
                self.guard_zones[1].enabled
            );
        }

        // No automatic target acquisition without guard zones
        if !guard_zone_active {
            // Complete any in-progress blobs before returning
            if !self.blobs_in_progress.is_empty() {
                completed_blobs = self.blobs_in_progress.drain(..).collect();
            }
            self.prev_bearing = bearing;
            return completed_blobs;
        }

        // Handle bearing wraparound - complete all blobs when we wrap
        if !self.blobs_in_progress.is_empty() {
            let first_blob_bearing = self.blobs_in_progress[0].min_bearing;
            // If we've wrapped around and are back near where blobs started, complete them all
            if bearing.raw() < first_blob_bearing.raw() && self.prev_bearing.raw() > bearing.raw() {
                completed_blobs.extend(self.blobs_in_progress.drain(..));
            }
        }

        // Filter pixels by guard zone and group into contiguous runs
        // A run is a sequence of pixels where each is adjacent (r differs by 1)
        let mut runs: Vec<Vec<i32>> = Vec::new();
        let mut current_run: Vec<i32> = Vec::new();

        // Log guard zone range checking periodically (every 256 bearings, on first strong pixel)
        if bearing.raw() % 256 == 0 && !strong_pixels.is_empty() {
            let first_r = strong_pixels[0];
            let last_r = *strong_pixels.last().unwrap();
            log::debug!(
                "GZ range check: spoke pixels {}..{}, gz0 range={}..{} ({}), gz1 range={}..{} ({})",
                first_r,
                last_r,
                self.guard_zones[0].inner_range,
                self.guard_zones[0].outer_range,
                if self.guard_zones[0].enabled { "on" } else { "off" },
                self.guard_zones[1].inner_range,
                self.guard_zones[1].outer_range,
                if self.guard_zones[1].enabled { "on" } else { "off" }
            );
        }

        for &r in strong_pixels {
            // Check if pixel is in a guard zone
            let in_zone = self.guard_zones[0].contains(bearing, r, spokes, heading)
                || self.guard_zones[1].contains(bearing, r, spokes, heading);
            if !in_zone {
                // End current run if any
                if !current_run.is_empty() {
                    runs.push(std::mem::take(&mut current_run));
                }
                continue;
            }

            // Check if this pixel is adjacent to the last pixel in current run
            if current_run.is_empty() || r == current_run.last().unwrap() + 1 {
                current_run.push(r);
            } else {
                // Start a new run
                runs.push(std::mem::take(&mut current_run));
                current_run.push(r);
            }
        }
        // Don't forget the last run
        if !current_run.is_empty() {
            runs.push(current_run);
        }

        let mut run_assigned: Vec<bool> = vec![false; runs.len()];
        let prev_bearing = bearing.sub(1, spokes);

        // For each run, find ALL adjacent blobs (there may be multiple that need merging)
        for (run_idx, run) in runs.iter().enumerate() {
            let mut adjacent_blob_indices: Vec<usize> = Vec::new();

            for (blob_idx, blob) in self.blobs_in_progress.iter().enumerate() {
                // Only consider blobs whose last_bearing is the previous spoke
                if blob.last_bearing() != prev_bearing {
                    continue;
                }
                // Check if any pixel in the run is adjacent to the blob
                for &r in run {
                    if blob.is_adjacent(r) {
                        adjacent_blob_indices.push(blob_idx);
                        break;
                    }
                }
            }

            if adjacent_blob_indices.is_empty() {
                continue;
            }

            run_assigned[run_idx] = true;

            // If multiple blobs are adjacent to this run, merge them all into the first one
            let primary_idx = adjacent_blob_indices[0];

            // First, merge any additional blobs into the primary blob
            // Process in reverse order to preserve indices during removal
            for &merge_idx in adjacent_blob_indices.iter().skip(1).rev() {
                let merge_blob = self.blobs_in_progress.remove(merge_idx);
                let primary = &mut self.blobs_in_progress[if merge_idx < primary_idx {
                    primary_idx - 1
                } else {
                    primary_idx
                }];
                // Merge the blob data - use raw values for min comparison since SpokeBearing
                // doesn't have a natural ordering (it wraps around)
                if merge_blob.min_bearing.raw() < primary.min_bearing.raw() {
                    primary.min_bearing = merge_blob.min_bearing;
                }
                primary.min_r = min(primary.min_r, merge_blob.min_r);
                primary.max_r = max(primary.max_r, merge_blob.max_r);
                primary.pixel_count += merge_blob.pixel_count;
                // Note: last_spoke_ranges from merged blob are from prev spoke, not needed
            }

            // Now extend the primary blob with this run
            // Need to recalculate primary_idx as it may have changed due to removals
            let adjusted_primary_idx = adjacent_blob_indices[0]
                - adjacent_blob_indices
                    .iter()
                    .skip(1)
                    .filter(|&&i| i < adjacent_blob_indices[0])
                    .count();
            let blob = &mut self.blobs_in_progress[adjusted_primary_idx];
            blob.start_new_spoke(bearing);
            for &r in run {
                blob.add_pixel(bearing, r);
                blob.push_last_spoke_range(r);
            }
        }

        // Start new blobs for unassigned runs
        for (run_idx, run) in runs.iter().enumerate() {
            if run_assigned[run_idx] {
                continue;
            }
            // Create a new blob with all pixels in this run
            // Store heading so we can validate guard zone membership when blob completes
            let mut blob = BlobInProgress::new(bearing, run[0], time, pos.clone(), heading);
            for &r in run.iter().skip(1) {
                blob.add_pixel(bearing, r);
                blob.push_last_spoke_range(r);
            }
            self.blobs_in_progress.push(blob);
        }

        // Find completed blobs: those that weren't extended this spoke
        // AND whose last_bearing is before the previous spoke (so they had a gap)
        let mut completed_indices: Vec<usize> = Vec::new();
        for (idx, blob) in self.blobs_in_progress.iter().enumerate() {
            // Blob is complete if it wasn't extended and last_bearing < prev_bearing
            // (meaning there's been at least one spoke with no contribution)
            if blob.last_bearing() != bearing && blob.last_bearing() != prev_bearing {
                completed_indices.push(idx);
            }
        }

        // Process completed blobs (in reverse order to preserve indices during removal)
        for &idx in completed_indices.iter().rev() {
            let blob = self.blobs_in_progress.remove(idx);
            completed_blobs.push(blob);
        }

        self.prev_bearing = bearing;
        completed_blobs
    }

    /// Complete all blobs in progress (called on angle wraparound or range change)
    pub(crate) fn complete_all_blobs(&mut self) -> Vec<BlobInProgress> {
        self.blobs_in_progress.drain(..).collect()
    }
}
