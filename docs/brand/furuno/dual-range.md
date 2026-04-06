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

In the native DLL, `radarNo` is delivered as a separate field in `RadarSweepMetadata`
(struct order: angle, heading, headingFlag, **radarNo**, range, scale, sweepLength, status)
and as the first parameter of the echo callback. The DLL must extract this from the raw UDP
frame to populate it. In our `parse_metadata_header()`, `data[12]` is the wire_index (range)
and `data[13]` is unexamined — this is the most likely position for the range identifier
(radarNo), since it sits adjacent to the range field and would be 0 for all non-dual-range
radars. A pcap capture from a DRS-NXT in dual range mode would confirm this.

### Reports (TCP)

Report responses (`$N62,...`, `$N63,...` etc.) arrive on the shared TCP connection. The
response likely includes the range context in its parameters, similar to how the unit field
is already present in range reports.

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

The UDP frame header must contain a range identifier. Based on our reverse engineering:

- TimeZero's `SweepSeriousFactory.CreateSweep()` receives `radarNo` as the first parameter
- The native DLL's `RmGetSweeps()` returns `RadarSweepMetadata` with a `radarNo` field
- In our `parse_metadata_header()`, candidates are:
  - `data[11]` bits 5-6 (`v2` and `v3`) — currently unused
  - `data[13]` or `data[14]` — not yet parsed
  - The `status` field that TimeZero extracts

Until confirmed with a real dual-range pcap, the demux logic should check the identified
byte and route `radarNo=0` frames to `common_a` and `radarNo=1` frames to `common_b`.

### Discovery Changes

The `FurunoLocator` creates two `RadarInfo` instances when the model name contains "NXT".
The `FurunoReportReceiver` is constructed with both and uses `set_range_b()` to install
the second `CommonRadar`.

### Non-Dual-Range Radars

For models that don't support dual range (DRS4DL, FAR series, etc.), the architecture
remains unchanged: single `RadarInfo`, single `CommonRadar`, `dual_range_id` always 0.
The `common_b` field is `None`.
