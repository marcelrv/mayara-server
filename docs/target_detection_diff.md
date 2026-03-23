# Comparison: radar_pi vs mayara-server Target Tracking

This document compares the ARPA/MARPA target tracking implementations between the OpenCPN radar_pi plugin (C++) and mayara-server (Rust).

## Kalman Filter

| Aspect | radar_pi (C++) | mayara-server (Rust) |
|--------|----------------|---------------------|
| **State vector** | 4-state: [lat_m, lon_m, dlat_dt, dlon_dt] | Same 4-state |
| **Coordinates** | Local meters from radar position | Same approach |
| **Process noise (Q)** | `NOISE = 0.015` (m/s)² | Same: `PROCESS_NOISE = 0.015` |
| **Measurement noise (R)** | Separate: angle=100, radius=25 | Position-based: 25.0 m² for both axes |
| **Initial P** | P(0,0)=P(1,1)=20, P(2,2)=P(3,3)=4 | Same values |
| **Measurement update** | `SetMeasurement()` in polar coords (angle, radius) | `update()` in geographic coords |
| **Observation matrix H** | Non-linear (angle/radius), Jacobian computed | Linear (position-only) |

**Key difference**: radar_pi uses polar coordinates (angle + radius) for measurement updates, computing a Jacobian for the non-linear observation. mayara-server uses direct geographic positions, which is simpler but may lose some geometric information.

## Target Matching

| Aspect | radar_pi | mayara-server |
|--------|----------|---------------|
| **Search method** | Pixel-based contour search around predicted position | Distance-based threshold |
| **Match distance** | `dist1 = speed * rotation_period * pixels_per_meter` | `uncertainty * 2.0` or `MAX_MATCH_DISTANCE_M` (150m) |
| **Timing** | Waits for beam to pass target angle | Processes blobs as they arrive |
| **Multi-pass search** | 2 passes: tighter first, then wider | Single pass |
| **Contour validation** | Rejects if contour length changes >2x | No contour validation |

## Target Lifecycle

| Aspect | radar_pi | mayara-server |
|--------|----------|---------------|
| **States** | LOST → ACQUIRE0 → ACQUIRE1 → ... → STATUS_TO_OCPN | Acquiring → Active |
| **Promotion threshold** | ~3 consecutive hits (ACQUIRE0→ACQUIRE1→...) | 2 hits (one acquiring match) |
| **Lost count** | `MAX_LOST_COUNT` misses before removal | 5 revolutions without match |
| **Contour averaging** | `WEIGHT_FACTOR = 0.1` exponential smoothing | Not implemented |

## Speed Calculation

| Aspect | radar_pi | mayara-server |
|--------|----------|---------------|
| **Initial speed** | "Forced position" method for status < 8 | Direct measurement on first update |
| **Forced factor** | `0.8^(status-1)` blending factor | N/A |
| **Speed limit** | `speed * 1.5` rejection | Implicit via `max_target_speed_ms` |
| **Turn limit** | Rejects >130° turns at >5 m/s for status<5 | Not implemented |

## Notable radar_pi Features Not in mayara-server

1. **Multi-pass target search**: radar_pi does LAST_PASS=2 passes, increasing search radius for struggling targets
2. **Contour length validation**: Rejects matches where contour size changes dramatically
3. **Turn rate validation**: Rejects physically impossible maneuvers (fast turns)
4. **Small-and-fast target handling**: Special logic for small targets at high speed
5. **Pixel counting**: Analyzes Doppler approaching/receding pixel counts
6. **Multi-radar handoff**: `FindBestRadarForTarget()` switches radar as target moves
7. **AIVDM/TTM output**: Sends AIS-style messages to OCPN

## mayara-server Additions

1. **Guard zone categorization**: `CandidateSource` tracks how candidate was detected
2. **MARPA uncertainty**: Custom initial Kalman uncertainty for manual acquisition
3. **Simplified blob-based detection**: Uses pre-detected blobs rather than contour walking

## Architecture Difference

The main architectural difference is that radar_pi searches raw spoke pixel data for contours, while mayara-server receives pre-processed blobs from the `BlobDetector`. This is a cleaner separation of concerns but means some low-level optimizations (like multi-pass searching on the pixel grid) aren't directly applicable.

## References

- radar_pi: https://github.com/opencpn-radar-pi/radar_pi
- Kalman Filter: "An Introduction to the Kalman Filter" by Greg Welch and Gary Bishop, TR45-041
