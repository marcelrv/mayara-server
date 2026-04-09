# Furuno DRS Dual Range Architecture

## Background

Furuno DRS-NXT radars (DRS4D-NXT, DRS6A-NXT, DRS12A-NXT, DRS25A-NXT) support dual range
mode, where a single physical antenna simultaneously serves two independent range/gain/clutter
contexts. The antenna alternates between the two range settings, producing interleaved spoke
data tagged with a range identifier (0 = Range A, 1 = Range B).

This was confirmed by reverse engineering TimeZero's `Fec.FarApi.dll` and native `radar.dll`
(see `research/furuno/timezero-dual-range.md`).

## Wire Protocol

### Commands

Every control command uses the same ASCII format, with the **dual range ID as the last parameter**:

```
$S62,<wire_index>,<unit>,<dual_range_id>\r\n     # Range
$S69,<status>,<wman>,...,<dual_range_id>\r\n      # TX/Standby
$S63,<auto>,<value>,...,<dual_range_id>\r\n        # Gain
```

- `dual_range_id = 0` targets Range A
- `dual_range_id = 1` targets Range B

There is no special "enable dual range" command. Dual range is implicitly active when the
radar firmware receives commands with `dual_range_id = 1`.

### Transmit Coupling

On DRS models, TX/Standby is coupled: the antenna transmits for both ranges simultaneously.
Setting TX on either range also sets it on the other. This means a single Power control can
be shared between both ranges.

### Spoke Data (UDP)

Both ranges arrive on the **same UDP multicast address** (`239.255.0.2:10024`), interleaved.
Each UDP frame has a 16-byte metadata header. Based on the TimeZero `RadarSweepMetadata`
struct and the native DLL callback signature, each frame carries a `radarNo` field (0 or 1)
that identifies which range the spokes belong to.

Live on-wire captures from a DRS4D-NXT confirmed that the dual-range identifier is at
**byte 15 bit 6** of the UDP frame header: Range A frames have byte 15 = 0x09, Range B
frames have byte 15 = 0x49. The radar.dll disassembly originally pointed at byte 11
bits 6-7 (our old `v3`), but on-wire captures show those bits are always `0b11` on
DRS4D-NXT (and `0b01` on DRS4W) regardless of which range a spoke belongs to — they are
model metadata, not the range selector. The value at byte 15 bit 6 is extracted as
`radar_no` and used for spoke demultiplexing (0 = Range A, 1 = Range B).

### Reports (TCP)

Report responses (`$N62,...`, `$N63,...`, etc.) arrive on the shared TCP connection and
are routed to Range A or Range B by extracting the per-command `drid` field. The
position of `drid` within the response varies per command:

- Status (`$N69`) — last field
- Gain (`$N63`) — last field
- Sea (`$N64`) — last field
- Rain (`$N65`) — field 4
- Tune (`$N75`) — field 2 (`screen` is `drid`)
- Range (`$N62`) — field 2 (after wire index and wire unit)

These positions were verified against live Wireshark captures from a DRS4D-NXT running
in dual range mode with TimeZero as the controlling client.

## Architecture: Navico HALO vs Furuno DRS

### Navico HALO Dual Range

Navico's approach relies on network-level separation:

- The beacon provides **separate addresses** for Range A and Range B (spoke data, reports,
  commands) in the `NavicoBeaconDual` struct
- Two completely independent `RadarInfo` + `NavicoReportReceiver` + `Command` instances
- Each listens on its own UDP multicast group for spokes
- Each has its own UDP socket for commands
- No shared state beyond the physical antenna

### Furuno DRS Dual Range

Furuno uses a single-connection model:

- **One TCP connection** for both ranges (commands and reports)
- **One UDP multicast address** for all spokes (both ranges interleaved)
- Range identity is encoded in the data stream, not the transport layer
- Commands append `dual_range_id` to distinguish targets

This means we cannot simply create two independent receiver instances like Navico does.

## Proposed Implementation

### Two RadarInfo Instances, One Receiver

```
                  FurunoLocator
                       |
                  login_to_radar()
                       |
            +----------+----------+
            v                     v
       RadarInfo A           RadarInfo B
       (dual="A")           (dual="B")
       own controls         own controls
       own message_tx       own message_tx
            |                     |
            +--------|------------+
                     v
          FurunoReportReceiver (single instance)
                     |
          +----------+----------+
          v                     v
    TCP connection         UDP socket
    (single stream)        (single multicast)
          |                     |
    read: parse $N        read: demux frames
    reports, route to     by radarNo field,
    correct RadarInfo     route to correct
          |               message_tx
          |                     |
    write: Command        +-----------+
    appends               v           v
    dual_range_id       Range A     Range B
    to each cmd         spokes      spokes
```

### What the Receiver Holds

The single `FurunoReportReceiver` manages:

- **TCP stream** (split into reader + writer as today)
- **UDP socket** (same as today)
- **Two `CommonRadar` instances**: `common_a` and `common_b`, each connected to its
  respective `RadarInfo`. Spokes are routed to the correct one based on the frame's
  range identifier.
- **One `Command` sender**: holds the TCP write half. Each command method accepts a
  `dual_range_id` parameter. The `Command` struct needs access to both `SharedControls`
  to read per-range state.

### Control Routing

When a client sets a control on Range A's `RadarInfo`, the control update arrives via
Range A's `control_update_rx` channel. The receiver identifies which range it belongs to
and calls `command_sender.set_control(cv, dual_range_id=0)`.

Similarly, Range B's updates use `dual_range_id=1`.

### Shared vs Independent Controls

**Shared (physical antenna properties):**
- Power / TX-Standby (coupled on DRS hardware)
- Antenna height
- Scan speed
- Blind sectors (NoTransmitSector1/2)
- Installation settings (firmware version, serial number, etc.)

**Independent per range:**
- Range
- Range Units
- Gain
- Sea clutter
- Rain clutter
- Noise rejection
- Interference rejection
- Target separation (RezBoost)
- Bird mode
- Doppler (Target Analyzer)

### Spoke Demultiplexing

The UDP frame header carries the range identifier at **byte 15 bit 6**
(`(data[15] & 0x40) >> 6`). `parse_metadata_header()` extracts this as `radar_no`;
`process_frame()` routes frames with `radar_no == 0` to the Range A `CommonRadar` and
frames with `radar_no == 1` to the Range B `CommonRadar`. Confirmed by alternating
dual-range frames in a live DRS4D-NXT capture.

### Discovery Changes

The `FurunoLocator` creates two `RadarInfo` instances when the model name contains "NXT".
The `FurunoReportReceiver` is constructed with both and uses `set_range_b()` to install
the second `CommonRadar`.

### Non-Dual-Range Radars

For models that don't support dual range (DRS4DL, FAR series, etc.), the architecture
remains unchanged: single `RadarInfo`, single `CommonRadar`, `dual_range_id` always 0.
The `common_b` field is `None`.
