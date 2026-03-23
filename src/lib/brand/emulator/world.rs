use std::f64::consts::PI;

use crate::radar::GeoPosition;

// Constants for target simulation
const TARGET_SPEED_KNOTS: f64 = 5.0;
const TARGET_DISTANCE_SOUTH: f64 = 500.0; // meters
const TARGET_SPACING: f64 = 500.0; // meters between targets
const NUM_TARGETS: usize = 5;

// Land area dimensions
const LAND_DISTANCE_NE: f64 = 2000.0; // 2 km NE of initial position
const LAND_WIDTH: f64 = 200.0; // meters (NW-SE axis)
const LAND_LENGTH: f64 = 1000.0; // meters (NE-SW axis)
const LAND_BEARING: f64 = 45.0; // degrees from North (NE)

// Conversion constants
const KNOTS_TO_MS: f64 = 1852.0 / 3600.0; // 1 knot = 1852m/h = 0.5144 m/s
const DEG_TO_RAD: f64 = PI / 180.0;

/// A moving target (boat)
#[derive(Clone, Debug)]
pub struct Target {
    /// Current position
    pub position: GeoPosition,
    /// Heading in degrees (0 = North, 90 = East)
    pub heading: f64,
    /// Speed in m/s
    pub speed: f64,
}

impl Target {
    fn new(position: GeoPosition, heading: f64, speed_knots: f64) -> Self {
        Target {
            position,
            heading,
            speed: speed_knots * KNOTS_TO_MS,
        }
    }

    fn update(&mut self, elapsed_secs: f64) {
        let distance = self.speed * elapsed_secs;
        let heading_rad = self.heading * DEG_TO_RAD;
        self.position = self.position.position_from_bearing(heading_rad, distance);
    }
}

/// Land area (stationary oblong)
#[derive(Clone, Debug)]
pub struct LandArea {
    /// Center position
    pub center: GeoPosition,
    /// Half-width (perpendicular to orientation)
    pub half_width: f64,
    /// Half-length (along orientation)
    pub half_length: f64,
    /// Orientation in radians (angle of the long axis from North)
    pub orientation_rad: f64,
}

impl LandArea {
    fn new(center: GeoPosition, width: f64, length: f64, orientation_deg: f64) -> Self {
        LandArea {
            center,
            half_width: width / 2.0,
            half_length: length / 2.0,
            orientation_rad: orientation_deg * DEG_TO_RAD,
        }
    }

    /// Check if a point is inside the land area
    fn contains(&self, point: &GeoPosition) -> bool {
        // Calculate distance and bearing from center to point
        let (distance, bearing) = distance_and_bearing(&self.center, point);

        // Rotate the bearing by the negative of orientation to align with local coordinates
        let local_angle = bearing - self.orientation_rad;

        // Calculate local x, y coordinates
        let local_x = distance * local_angle.sin(); // perpendicular to long axis
        let local_y = distance * local_angle.cos(); // along long axis

        // Check if within the oblong bounds
        local_x.abs() <= self.half_width && local_y.abs() <= self.half_length
    }
}

/// The simulated world
pub struct EmulatorWorld {
    /// Land area (fixed position)
    pub land: LandArea,
    /// Moving targets
    pub targets: Vec<Target>,
}

impl EmulatorWorld {
    pub fn new(initial_boat_pos: GeoPosition) -> Self {
        // Create land area 2km NE of initial position
        let land_center =
            initial_boat_pos.position_from_bearing(LAND_BEARING * DEG_TO_RAD, LAND_DISTANCE_NE);
        // Orientation: NE-SW axis means the long axis points at 45 degrees
        let land = LandArea::new(land_center, LAND_WIDTH, LAND_LENGTH, LAND_BEARING);

        // Create targets 500m south of initial boat position, moving East
        // They start at various positions West of the boat
        let mut targets = Vec::with_capacity(NUM_TARGETS);
        let south_bearing = 180.0 * DEG_TO_RAD; // South
        let west_bearing = 270.0 * DEG_TO_RAD; // West

        // Base position: 500m south of boat
        let base_pos = initial_boat_pos.position_from_bearing(south_bearing, TARGET_DISTANCE_SOUTH);

        for i in 0..NUM_TARGETS {
            // Offset West by i * TARGET_SPACING
            let offset = i as f64 * TARGET_SPACING;
            let target_pos = base_pos.position_from_bearing(west_bearing, offset);
            // Moving East at 5 knots
            targets.push(Target::new(target_pos, 90.0, TARGET_SPEED_KNOTS));
        }

        EmulatorWorld { land, targets }
    }

    /// Update all moving objects based on elapsed time
    pub fn update(&mut self, elapsed_secs: f64) {
        for target in &mut self.targets {
            target.update(elapsed_secs);
        }
    }

    /// Get the radar return intensity at a given position
    /// Returns 0-15 intensity value (0 = no return, 15 = strongest)
    pub fn get_intensity(&self, boat_pos: &GeoPosition, bearing_rad: f64, distance: f64) -> u8 {
        // Calculate the world position at this bearing/distance from boat
        let point = boat_pos.position_from_bearing(bearing_rad, distance);

        // Check land
        if self.land.contains(&point) {
            return 14; // Strong return for land
        }

        // Check targets - they appear as point targets with some spread
        const TARGET_RADIUS: f64 = 30.0; // meters - radar target size
        for target in &self.targets {
            let target_distance = distance_between(&point, &target.position);
            if target_distance < TARGET_RADIUS {
                // Intensity decreases with distance from target center
                let intensity = 15.0 - (target_distance / TARGET_RADIUS) * 5.0;
                return intensity.max(13.0) as u8;
            }
        }

        0 // No return
    }
}

/// Calculate distance in meters between two positions
fn distance_between(from: &GeoPosition, to: &GeoPosition) -> f64 {
    const EARTH_RADIUS: f64 = 6_371_000.0; // meters

    let lat1 = from.lat().to_radians();
    let lat2 = to.lat().to_radians();
    let delta_lat = (to.lat() - from.lat()).to_radians();
    let delta_lon = (to.lon() - from.lon()).to_radians();

    let a =
        (delta_lat / 2.0).sin().powi(2) + lat1.cos() * lat2.cos() * (delta_lon / 2.0).sin().powi(2);
    let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());

    EARTH_RADIUS * c
}

/// Calculate distance and bearing from one position to another
/// Returns (distance in meters, bearing in radians)
fn distance_and_bearing(from: &GeoPosition, to: &GeoPosition) -> (f64, f64) {
    let distance = distance_between(from, to);

    let lat1 = from.lat().to_radians();
    let lat2 = to.lat().to_radians();
    let delta_lon = (to.lon() - from.lon()).to_radians();

    let y = delta_lon.sin() * lat2.cos();
    let x = lat1.cos() * lat2.sin() - lat1.sin() * lat2.cos() * delta_lon.cos();
    let bearing = y.atan2(x);

    // Normalize bearing to [0, 2*PI)
    let bearing = (bearing + 2.0 * PI) % (2.0 * PI);

    (distance, bearing)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_land_area_contains() {
        let center = GeoPosition::new(53.2, 5.3);
        let land = LandArea::new(center, 200.0, 1000.0, 45.0);

        // Center should be inside
        assert!(land.contains(&center));

        // Point far away should be outside
        let far = GeoPosition::new(54.0, 6.0);
        assert!(!land.contains(&far));
    }

    #[test]
    fn test_target_movement() {
        let pos = GeoPosition::new(53.0, 5.0);
        let mut target = Target::new(pos, 90.0, 5.0); // Moving East at 5 knots

        // After 1 hour, should move ~9.26 km East
        target.update(3600.0);

        // Longitude should increase (moving East)
        assert!(target.position.lon() > 5.0);
    }

    #[test]
    fn test_distance_between() {
        let p1 = GeoPosition::new(53.0, 5.0);
        let p2 = GeoPosition::new(53.0, 5.01); // Small offset East

        let dist = distance_between(&p1, &p2);
        // At 53N, 0.01 degrees longitude is about 670m
        assert!(dist > 600.0 && dist < 700.0);
    }
}
