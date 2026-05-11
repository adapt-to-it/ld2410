# HLK-LD2420 — radar→host data-frame format

**Companion to** [`HLK-LD2420_protocol.md`](HLK-LD2420_protocol.md) (which
covers only the host→radar command side, as the source Hi-Link XLSX
explicitly scopes itself).

> **Provenance.** Hi-Link does **not** ship the data-frame layout in
> the V2.2 command XLSX. The byte-level details below are
> cross-checked against:
>
> 1. **ESPHome `ld2420` component** ([`esphome/esphome`](https://github.com/esphome/esphome) `components/ld2420/`)
>    — production C++ driver validated against real LD2420 hardware
>    in the ESPHome user community for years. Authoritative for the
>    energy-frame layout; this document is byte-for-byte from
>    `ld2420.cpp` `handle_energy_mode_()` and the comment block above
>    it.
> 2. **Hi-Link LD2420 product manual PDF** — not redistributed in this
>    repo; the canonical community mirror is
>    [`soubhik-khan/HLK-LD2420`](https://github.com/soubhik-khan/HLK-LD2420).
>    The product manual describes the data-output frame format in
>    plain text. When this document and the product manual disagree,
>    the product manual wins — open an issue with a captured frame.
>
> This layout has been verified against running silicon by third
> parties but **not** by the maintainers of this fork (we do not have
> an LD2420 sample on the bench yet — see banner in
> `src/ld2420_variants/ld2420_v22.h`).

## On-wire envelope

The LD2420 data-path envelope is identical to the LD2410 family's:

| Field | Width | Wire bytes | Little-endian uint32 |
|---|---|---|---|
| Frame head | 4 | `F4 F3 F2 F1` | `0xF1F2F3F4` |
| Intra-frame length | 2 | `LL LL` | LE16 — value = bytes-between-head-and-tail |
| Intra-frame data | N | … | mode-specific (see below) |
| Frame tail | 4 | `F8 F7 F6 F5` | `0xF5F6F7F8` |

> **Note on the byte-reversed claim.** An earlier version of this
> repository's documentation claimed the LD2420 data envelope was
> "byte-reversed" relative to LD2410's (`F1 F2 F3 F4` / `F5 F6 F7 F8`).
> That was a misreading of ESPHome's `static constexpr uint32_t
> ENERGY_FRAME_HEADER = 0xF1F2F3F4;` constant — when stored
> little-endian (as constants always are on the target cores) the
> resulting on-wire bytes are `F4 F3 F2 F1`. Both chip families share
> the same data-envelope magic. The frames remain distinguishable
> because their **intra-frame content** is incompatible (LD2410 reports
> classified presence/distance, LD2420 reports raw FFT energies).

## Energy frame (transparent / energy output mode)

**Total wire size: 45 bytes.** Length field value: `0x0023` (= 35
intra-frame bytes between the head and tail).

| Offset | Width | Field | Type | Notes |
|---|---|---|---|---|
| 0 | 4 | head | `uint8_t[4]` | `F4 F3 F2 F1` |
| 4 | 2 | intra_len | LE `uint16_t` | always `0x0023` for energy frames |
| 6 | 1 | presence | `uint8_t` | `0x00` = no target, `0x01` = target present |
| 7 | 2 | distance_cm | LE `uint16_t` | target distance, in centimetres |
| 9 | 32 | gate_energies | LE `uint16_t[16]` | per-gate FFT energy, one 2-byte value per range gate (gates 0..15) |
| 41 | 4 | tail | `uint8_t[4]` | `F8 F7 F6 F5` |

### Field semantics

- **`presence`** is a boolean classifier produced by the radar's
  on-chip ABD (Adaptive Background Detection) pipeline. It is `0x01`
  when ABD currently considers a target detected, `0x00` otherwise.
  The host can implement its own classifier on top of the raw gate
  energies if a different policy is wanted (e.g. tighter thresholds,
  per-gate hold timers, occupancy mode).
- **`distance_cm`** is the radar's best estimate of the closest
  detected target's distance, in centimetres. Valid only when
  `presence` is `0x01`; the field may contain stale data otherwise.
- **`gate_energies[i]`** is the squared 2DFFT modulus at range gate
  `i` (where each gate represents a discrete range bin — see the
  product manual for the gate-to-distance conversion, which depends
  on the firmware-configured chirp parameters). Higher values mean
  more reflected energy; the ABD thresholds (configured via the
  `0x0007 LD2420_ABD_W_HIGH_THRESHOLD` / `LD2420_ABD_W_LOW_THRESHOLD`
  commands — see [`HLK-LD2420_protocol.md`](HLK-LD2420_protocol.md) §1.2.4)
  decide which `gate_energies[i]` values count as a "hit".

### Mode selection

The energy frame is emitted while the radar's `systemMode` system
parameter (`LD2420_SYS_W_MODE`, see protocol doc §1.2.7 Table 7) is
set to a value that produces per-gate energy output. The V2.2 XLSX
documents four named modes (`transparent`, `MTT`, `VS`, `GR`) but
does **not** describe the wire format of each — the energy-frame
layout above is what production drivers (ESPHome) observe when the
mode is configured to emit raw energies. On firmwares supporting it,
ESPHome reports `CMD_SYSTEM_MODE_ENERGY = 0x0004` as the mode value
that produces this format.

If you have an LD2420 and observe a different frame layout for a
given `systemMode`, please open an issue with a captured frame dump.

## Other data-frame types (not yet documented)

The product manual describes additional output modes (1DFFT data,
2DFFT data, 2DFFT peak, DSRAW) selectable through the
`0x0024 / Enter factory test mode` command's data-type field
(`LD2420_FT_DATA_TYPE_*`). These are not in scope for this driver
yet — the host-side classifier built on the energy-frame layout
above is sufficient for most presence/distance applications.

## Cross-references

- [`HLK-LD2420_protocol.md`](HLK-LD2420_protocol.md) — host→radar
  command side (V2.2 XLSX transcription).
- [`10-api-ld2420.md`](10-api-ld2420.md) — public API for the
  `class ld2420` driver, including the `snapshotTargetState()`
  atomic-read helper for the fields decoded above.
- [`ld2420-method-coverage.md`](ld2420-method-coverage.md) — driver
  method coverage matrix.
- ESPHome reference implementation:
  <https://github.com/esphome/esphome/tree/dev/esphome/components/ld2420>
- Community PDF mirror (`soubhik-khan/HLK-LD2420`):
  <https://github.com/soubhik-khan/HLK-LD2420>
