# Garmin Radar Protocol Specification

This document describes the network protocol used by Garmin marine radars, derived from analysis of the OpenCPN radar_pi plugin.

## Supported Models

| Model | Spokes | Sample Depth | Max Range |
|-------|--------|--------------|-----------|
| **Garmin HD** | 720 per rotation | 1-bit (binary) | 48 NM |
| **Garmin xHD** | 1440 per rotation | 8-bit (0-255) | 48 NM |

The HD series uses binary (on/off) returns packed 8 per byte, while the xHD provides full 8-bit intensity values per sample.

> **Note**: Newer Garmin radars (xHD2, Fantom series) use a different protocol and are **not supported**. The protocol documented here applies only to the original HD and xHD models.

## Network Configuration

### Addressing

Garmin radars operate on a fixed private network:

| Parameter | Value |
|-----------|-------|
| Radar subnet | 172.16.2.0/24 |
| Interface match | 172.16.0.0/12 (netmask 255.240.0.0) |
| Report multicast | 239.254.2.0:50100 |
| Data multicast (xHD) | 239.254.2.0:50102 |
| Control port | 50101 (unicast to radar IP) |

### Discovery

1. Enumerate network interfaces
2. Find interface with IP matching 172.16.0.0/12 subnet
3. Join multicast group 239.254.2.0 on that interface
4. Listen for status reports (packet type 0x2a5 for HD, various for xHD)
5. Extract radar's IP address from received packets
6. Send control commands to radar IP on port 50101

## Spoke Data Format

### Garmin HD

HD packs 4 spokes per UDP packet with 1-bit samples (8 samples per byte). Spoke data and status reports share the same multicast address/port (239.254.2.0:50100).

```
Offset  Size  Field              Description
------  ----  -----              -----------
0       4     packet_type        0x000002a3
4       4     len1               Packet data length
8       2     angle              Base spoke angle (×2 for actual)
10      2     scan_length        Total bytes for all 4 spokes
12      4     display_meters     Display range setting
16      4     range_meters       Actual radar range (add 1 to get true range)
20      4     gain_level[4]      Gain value per spoke
24      4     sea_clutter[4]     Sea clutter per spoke
28      4     rain_clutter[4]    Rain clutter per spoke
32      2     dome_offset        Bearing alignment (signed int16)
34      1     FTC_mode           Fast Time Constant mode
35      1     crosstalk_onoff    Interference rejection
36      2     fill_2             Reserved
38      2     fill_3             Reserved
40      4     timed_transmit[4]  Timed transmit settings
44      1     dome_speed         Scan speed
45      7     fill_4             Reserved
52      N     line_data          Spoke data (4 consecutive buffers)
```

**Sample decoding**: Each byte contains 8 samples packed as bits (LSB first). Unpack as:
```
for each byte:
  sample[0] = (byte & 0x01) ? 255 : 0
  sample[1] = (byte & 0x02) ? 255 : 0
  sample[2] = (byte & 0x04) ? 255 : 0
  ...
  sample[7] = (byte & 0x80) ? 255 : 0
```

**Spoke layout**: The `line_data` contains 4 consecutive spoke buffers, each `scan_length/4` bytes. Process each buffer separately to get 4 sequential spokes.

**Angle conversion**: `spoke_number = angle × 2 + spoke_offset` where spoke_offset is 0-3 for the 4 packed spokes.

**Range correction**: Add 1 to `range_meters` to get actual range: `actual_range = range_meters + 1`

**Resolution**: 720 spokes × 0.5° = 360°, max 252 bytes/spoke = 2016 samples/spoke

### Garmin xHD

xHD sends one spoke per UDP packet with 8-bit samples.

```
Offset  Size  Field              Description
------  ----  -----              -----------
0       4     packet_type        0x000002a3
4       4     len1               Packet data length
8       2     fill_1             Reserved
10      2     scan_length        Sample count
12      2     angle              Spoke angle (÷8 for actual)
14      2     fill_2             Reserved
16      4     range_meters       Actual radar range
20      4     display_meters     Display range setting
24      2     fill_3             Reserved
26      2     scan_length_bytes  Video bytes (short)
28      2     fill_4             Reserved
30      4     scan_length_bytes  Video bytes (integer)
34      2     fill_5             Reserved
36      N     line_data          Spoke data (519-705 bytes)
```

**Sample decoding**: Each byte is a direct intensity value 0-255.

**Angle conversion**: `spoke_number = angle / 8`

**Resolution**: 1440 spokes × 0.25° = 360°

## Control Commands

Commands are sent as UDP unicast to the radar's IP on port 50101.

### Packet Structure

```
// 9-byte command
struct command_9 {
    uint32_t packet_type;
    uint32_t len1;         // = 1
    uint8_t  parm1;
};

// 10-byte command
struct command_10 {
    uint32_t packet_type;
    uint32_t len1;         // = 2
    uint16_t parm1;
};

// 12-byte command
struct command_12 {
    uint32_t packet_type;
    uint32_t len1;         // = 4
    uint32_t parm1;
};
```

### Garmin HD Commands

| Function | Type | Packet Type | Parameter |
|----------|------|-------------|-----------|
| Transmit | 10 | 0x2b2 | 1=off, 2=on |
| Range | 12 | 0x2b3 | meters - 1 |
| Gain | 12 | 0x2b4 | 0-100 manual, 344=auto |
| Sea Clutter | 12+4 | 0x2b5 | level (parm1), mode (parm2) |
| Rain Clutter | 12 | 0x2b6 | 0-100 |
| Bearing Align | 10 | 0x2b7 | degrees offset |
| FTC Mode | 9 | 0x2b8 | FTC setting |
| Interference | 9 | 0x2b9 | 0-100 |
| Scan Speed | 9 | 0x2be | speed value |

### Garmin xHD Commands

| Function | Type | Packet Type | Parameter |
|----------|------|-------------|-----------|
| Transmit | 9 | 0x919 | 0=off, 1=on |
| Range | 12 | 0x91e | meters (200 to 88896) |
| Scan Speed | 9 | 0x916 | value × 2 |
| Gain Mode | 9 | 0x924 | 0=manual, 2=auto |
| Gain Level | 10 | 0x925 | value × 100 (0-10000) |
| Auto Gain Level | 9 | 0x91d | 0=low, 1=high |
| Bearing Align | 12 | 0x930 | degrees × 32 (signed) |
| Interference | 9 | 0x91b, 0x932, 0x2b9 | 0-100 (send all three) |
| Rain Mode | 9 | 0x933 | 0=off, 1=manual |
| Rain Level | 10 | 0x934 | value × 100 |
| Sea Mode | 9 | 0x939 | 0=off, 1=manual, 2=auto |
| Sea Level | 10 | 0x93a | value × 100 |
| Sea Auto Level | 9 | 0x93b | auto level (0+) |
| No-TX Enable | 9 | 0x93f | 0=off, 1=on |
| No-TX Start | 12 | 0x940 | degrees × 32 (signed, -180 to +180) |
| No-TX End | 12 | 0x941 | degrees × 32 (signed, -180 to +180) |
| Timed Idle Mode | 9 | 0x942 | 0=off, 1=on |
| Timed Idle Time | 10 | 0x943 | minutes × 60 |
| Timed Run Time | 10 | 0x944 | minutes × 60 |

**Multi-packet commands**:
- **Gain (auto)**: Send mode=2 via 0x924, then auto-level (0=low, 1=high) via 0x91d
- **Gain (manual)**: Send mode=0 via 0x924, then level via 0x925
- **Sea clutter (auto)**: Send mode=2 via 0x939, then auto-level via 0x93b
- **Sea clutter (manual)**: Send mode=1 via 0x939, then level via 0x93a
- **Sea clutter (off)**: Send mode=0 via 0x939
- **Rain clutter (manual)**: Send mode=1 via 0x933, then level via 0x934
- **Rain clutter (off)**: Send mode=0 via 0x933
- **No-transmit zone**: To enable, send enable=1 via 0x93f, then start via 0x940 and end via 0x941. To disable, send enable=0 via 0x93f.
- **Interference**: Send same value to all three packet types (0x91b, 0x932, 0x2b9)

## Status Reports

### Garmin HD Status (0x2a5)

The HD radar sends periodic status packets (packet type 0x2a5) and settings packets (0x2a7):

**Status packet (0x2a5):**
```
Offset  Size  Field              Description
------  ----  -----              -----------
0       4     packet_type        0x000002a5
4       4     len1               Packet data length
8       2     scanner_state      Current radar state (1,3,4,5)
10      2     warmup             Seconds until next state change
12      4     range_meters       Current range (add 1)
16      1     gain_level         Gain value
17      1     gain_mode          0=manual, 1=auto high, 2=auto low
18      2     fill_1             Reserved
20      1     sea_clutter_level  Sea clutter value
21      1     sea_clutter_mode   0=off, 1=manual, 2=auto
22      2     fill_2             Reserved
24      1     rain_clutter_level Rain clutter value
25      3     fill_3             Reserved
28      2     dome_offset        Bearing alignment (signed)
30      1     FTC_mode           FTC setting
31      1     crosstalk_onoff    Interference rejection
32      4     fill_4             Reserved
36      1     timed_transmit_mode      Timed TX mode
37      1     timed_transmit_transmit  TX duration
38      1     timed_transmit_standby   Standby duration
39      1     fill_5             Reserved
40      1     dome_speed         Scan speed
41      7     fill_6             Reserved
```

### Garmin xHD Status

xHD sends individual status packets per setting:

| Packet Type | Content |
|-------------|---------|
| 0x0916 | Scan speed (value >> 1) |
| 0x0919 | Transmit state (0=off, 1=on) |
| 0x091e | Range in meters |
| 0x0924 | Auto gain on/off (0=manual, 2=auto) |
| 0x0925 | Gain level (÷100 for 0-100 range) |
| 0x091d | Auto gain mode (0=low, 1=high) |
| 0x0930 | Bearing alignment (÷32 for degrees) |
| 0x0932 | Crosstalk/interference rejection |
| 0x0933 | Rain clutter mode (0=off, 1=manual) |
| 0x0934 | Rain clutter level (÷100) |
| 0x0939 | Sea clutter mode (0=off, 1=manual, 2=auto) |
| 0x093a | Sea clutter level (÷100) |
| 0x093b | Sea clutter auto level |
| 0x093f | No-transmit zone mode (0=off, 1=on) |
| 0x0940 | No-transmit zone start (÷32 for degrees) |
| 0x0941 | No-transmit zone end (÷32 for degrees) |
| 0x0942 | Timed idle mode (0=off, 1=on) |
| 0x0943 | Timed idle duration (÷60 for minutes) |
| 0x0944 | Timed run duration (÷60 for minutes) |
| 0x0992 | Scanner state (see Power States) |
| 0x0993 | State change countdown (÷1000 for seconds) |
| 0x099b | Error/info message (64-char string at offset 16) |

**Gain reporting sequence**: xHD reports gain in 3 packets sent every 2 seconds: 0x0924 (auto mode), 0x0925 (level), 0x091d (auto level). Combine them to determine full gain state.

## Power States

### Garmin HD

```
Status 1 = WARMING_UP
Status 3 = STANDBY
Status 4 = TRANSMIT
Status 5 = SPINNING_UP
```

### Garmin xHD

```
Status 2 = WARMING_UP
Status 3 = STANDBY
Status 4 = SPINNING_UP
Status 5 = TRANSMIT
Status 6 = STOPPING
Status 7 = SPINNING_DOWN
Status 10 = STARTING
```

### State Transitions

```
OFF → STANDBY (on detection)
STANDBY → STARTING → WARMING_UP → SPINNING_UP → TRANSMIT
TRANSMIT → STOPPING → SPINNING_DOWN → STANDBY
```

The transmit on/off commands initiate transitions. The radar reports intermediate states as it warms up or spins down.

## Range Values

### Garmin HD (Metric)

```
250, 500, 750, 1000, 1500, 2000, 3000, 4000,
6000, 8000, 12000, 16000, 24000, 36000, 48000, 64000 meters
```

### Garmin xHD (Mixed NM/Metric)

```
232, 463, 926, 1389, 1852, 2778, 3704, 5556,
7408, 11112, 14816, 22224, 29632, 44448, 66672, 88896 meters
```

Note: xHD values are based on nautical miles (1 NM = 1852m). The values shown are fractions/multiples of NM converted to meters.

## Key Differences: HD vs xHD

| Feature | HD | xHD |
|---------|----|----|
| Angular resolution | 0.5° (720 spokes) | 0.25° (1440 spokes) |
| Sample encoding | 1-bit binary | 8-bit intensity |
| Spokes per packet | 4 | 1 |
| Data port | 50100 (shared) | 50102 (dedicated) |
| Transmit command | 0x2b2, value 1/2 | 0x919, value 0/1 |
| Range command | 0x2b3, meters-1 | 0x91e, meters direct |
| Gain | Single command | Separate mode/level |
| FTC mode | Yes | No |
| No-transmit zones | No | Yes |
| Timed transmit | Limited | Full control |
| Status reporting | Single packet | Per-setting packets |

## Implementation Notes

### Byte Order

All multi-byte values are little-endian in the protocol.

### Watchdog

If no status packets are received within 2 seconds, the radar should be considered offline.

### Keep-Alive

Garmin radars do not require periodic keep-alive messages.

### Error Handling

- xHD may send malformed packets occasionally; count errors but continue processing
- Missing spokes should be tracked for diagnostics
- Invalid interface addresses should fall back to generic socket binding

## References

- OpenCPN radar_pi plugin: https://github.com/opencpn-radar-pi/radar_pi
- Implementation files:
  - `include/garminhd/garminhdtype.h`
  - `include/garminxhd/garminxhdtype.h`
  - `src/garminhd/GarminHDReceive.cpp`
  - `src/garminxhd/GarminxHDReceive.cpp`
  - `src/garminhd/GarminHDControl.cpp`
  - `src/garminxhd/GarminxHDControl.cpp`
