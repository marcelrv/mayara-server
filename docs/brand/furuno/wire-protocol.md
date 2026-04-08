# Furuno DRS Wire Protocol Reference

Derived from reverse engineering TimeZero's `Fec.FarApi.dll` and native `\FecDll_x64\radar.dll`.
See `research/furuno/` for the full decompilation analysis.

## Command Format

```
$<prefix><cmd_hex>,<param1>,<param2>,...\r\n
```

### Prefix Characters

From the string `"SRNXEO"` indexed by send mode:

| Char | Meaning |
|------|---------|
| S    | Set command (client to radar) |
| R    | Request current value |
| N    | Response from radar |

### Command IDs

Command IDs are `0x60 + offset`, transmitted as uppercase hex:

| Hex | Name | Format (Set) |
|-----|------|--------------|
| 60  | Connect | `$S60,...` |
| 61  | DispMode | `$S61,...` |
| 62  | Range | `$S62,<wire_index>,<unit>,<dual_range_id>` |
| 63  | Gain | `$S63,<auto>,<value>,0,80,<dual_range_id>` |
| 64  | Sea | `$S64,<auto>,<value>,50,0,0,<dual_range_id>` |
| 65  | Rain | `$S65,<auto>,<value>,0,0,0,<dual_range_id>` |
| 67  | SignalProcessing | `$S67,0,<feature>,<value>,0` |
| 69  | TX/Standby | `$S69,<status>,<wman>,<w_send>,<w_stop>,0,<dual_range_id>` |
| 77  | BlindSector | `$S77,<s2_enable>,<s1_start>,<s1_width>,<s2_start>,<s2_width>` |
| 83  | MainBangSize | `$S83,<value_0-255>,0` |
| 84  | AntennaHeight | `$S84,0,<meters>,0` |
| 89  | ScanSpeed | `$S89,<mode>,0` |
| 96  | Modules | `$R96` (query only) |
| E3  | AliveCheck | `$RE3` (query only) |
| ED  | BirdMode | `$SED,<level>,0` |
| EE  | RezBoost | `$SEE,<level>,0` |
| EF  | TargetAnalyzer | `$SEF,<enabled>,<mode>,0` |

### Range Command Details

```
$S62,<wire_index>,<unit>,<dual_range_id>
```

**Wire indices** are non-sequential. The radar uses specific values, not array positions:

| Wire idx | NM value | Wire idx | NM value |
|----------|----------|----------|----------|
| 21       | 1/16     | 11       | 12       |
| 0        | 1/8      | 12       | 16       |
| 1        | 1/4      | 13       | 24       |
| 2        | 1/2      | 14       | 32       |
| 3        | 3/4      | 19       | 36       |
| 4        | 1        | 15       | 48       |
| 5        | 1.5      | 20       | 64       |
| 6        | 2        | 16       | 72       |
| 7        | 3        | 17       | 96       |
| 8        | 4        | 18       | 120      |
| 9        | 6        |          |          |
| 10       | 8        |          |          |

Note: indices 19, 20 are out of sequence (36 and 64 were added later).

**Unit values:**

| Value | Unit |
|-------|------|
| 0     | NM (nautical miles) |
| 1     | km (kilometers) |
| 2     | SM (statute miles) |
| 3     | Kyd (kilo-yards) |

In km mode, the same wire indices represent km values instead of NM values.
Wire index 21 (0.0625) is not available in km mode.

**dual_range_id:** 0 = Range A, 1 = Range B.

## Discovery Protocol

### Beacon

- Listen on UDP `172.31.255.255:10010`
- Send discovery packets periodically (request beacon, request model, announce presence)
- Radar responds with two report types:
  1. Beacon report (32+ bytes) — contains radar hostname
  2. Model report (170 bytes) — contains model ID, firmware version, serial number

### TCP Login

1. Connect to radar IP port 10000
2. Send 56-byte COPYRIGHT message
3. Receive 8-byte header + 4-byte dynamic port number
4. All subsequent TCP communication uses this dynamic port

## Spoke Data (UDP)

- Multicast: `239.255.0.2:10024`
- Broadcast fallback: `172.31.255.255:10024`
- 8192 spokes per revolution
- Max 883 bytes per spoke

### Frame Format

16-byte header followed by compressed spoke data:

| Offset | Bits | Field |
|--------|------|-------|
| 0      | 8    | Frame type (0x02) |
| 1      | 8    | sequence_number |
| 2-3    | 16   | total_length (big-endian) |
| 4-7    | 32   | timestamp (little-endian) |
| 8-9    | 9+7  | spoke_data_len (bytes 8+9 bit 0), spoke_count (byte 9 bits 1-7) |
| 10-11  | 11+2+1+2 | sweep_len (byte 10 + byte 11 bits 0-2), encoding (byte 11 bits 3-4), heading_valid (byte 11 bit 5), model metadata (byte 11 bits 6-7: 0b11 on DRS4D-NXT, 0b01 on DRS4W) |
| 12     | 6+2  | wire_index (bits 0-5), range_status (bits 6-7) |
| 13     | 8    | range_resolution metadata |
| 14-15  | 11   | range_value (byte 14 + byte 15 bits 0-2) |
| 15     | 4    | echo_type (bits 4-5), **dual_range_id (bit 6)** — 0 = Range A, 1 = Range B |

Each spoke within the frame:
- 2 bytes: angle (0-8191, little-endian)
- 2 bytes: heading (little-endian)
- Variable: compressed echo data (encoding 0-3)

### Compression Encodings

| Encoding | Description |
|----------|------------|
| 0        | Raw (uncompressed) |
| 1        | Run-length encoded |
| 2        | First spoke: encoding 1, subsequent: delta from previous |
| 3        | Delta from previous spoke |
