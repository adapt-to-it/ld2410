# LD2420 — method coverage matrix

How the public API of `class ld2420` maps to the documented protocol
capabilities of the HLK-LD2420 V2.2 firmware. This file is the **source
of truth** for the `#ifdef LD2420_HAS_*` gates in
`src/ld2420.h` / `src/ld2420.cpp`.

When you add, remove, or rename a public method, update the relevant
row of Table 1 here and the corresponding `#ifdef` gate in the header.

References:
- LD2420 protocol: [`docs/HLK-LD2420_protocol.md`](HLK-LD2420_protocol.md)
- Variant header: `src/ld2420_variants/ld2420_v22.h`
- Sibling sensor LD2410 family coverage: [`docs/method-coverage.md`](method-coverage.md)

Legend:
- ✅ exposed via a public method
- 🟡 partial — wire path implemented, payload decoding not yet exposed
- ❌ documented but not exposed yet
- — not part of this firmware revision's protocol

---

## Table 1 — Commands (host → radar)

| Opcode | Section | Capability | Status | Public method | Feature flag |
|---|---|---|:-:|---|---|
| `0x00FF` | §1.2.13 | Open command mode | ✅ | `enterCommandMode()` (two-step open per §1.1.1; populates `protocol_version` + `tx_rx_buffer_size`) | `LD2420_HAS_CONFIGURATION_MODE` |
| `0x00FE` | §1.2.14 | Disable command mode | ✅ | `exitCommandMode()` | `LD2420_HAS_CONFIGURATION_MODE` |
| `0x0000` | §1.2.1 | Read version | ✅ | `requestFirmwareVersion()` → populates `firmware_version_ascii[16]` (NUL-terminated copy), `firmware_version_length`, and a best-effort numeric breakdown into `firmware_major_version` / `firmware_minor_version` / `firmware_patch_version` | `LD2420_HAS_READ_VERSION` |
| `0x0001` | §1.2.2 | Write register | ✅ | `writeRegister(uint16_t chip, uint16_t reg, uint16_t value)` — single-register only; bulk path can be added if a real use case lands. Wraps its own enter/exitCommandMode pair. | `LD2420_HAS_REGISTER_RW` |
| `0x0002` | §1.2.3 | Read register | ✅ | `readRegister(uint16_t chip, uint16_t reg, uint16_t & out)` — single-register only. The XLSX V2.2 "Read register" send-frame examples have the intra-length field two bytes short; the driver follows the consistent-with-everything-else convention (`len = full intra including cmd-word`). | `LD2420_HAS_REGISTER_RW` |
| `0x0007` | §1.2.4 | Configure ABD parameters | ❌ | _not exposed yet_ — payload is `(param_word (2B) + value (4B)) × N`; param words enumerated in `LD2420_ABD_W_*`. Planned: typed setters for global `roiMin`/`roiMax`/`delayTime` and per-block (high/low) frame counts + per-gate threshold packing | `LD2420_HAS_ABD_PARAMS` |
| `0x0008` | §1.2.5 | Read ABD parameters | ❌ | _not exposed yet_ — request is `(param_word (2B)) × N`; response is `(value (4B)) × N`. Per-gate threshold blocks at `LD2420_ABD_R_HIGH_THRESH_BASE` (0x0020 + gate) and `LD2420_ABD_R_LOW_THRESH_BASE` (0x0030 + gate) | `LD2420_HAS_ABD_PARAMS` |
| `0x0011` | §1.2.6 | Read serial number | ❌ | _not exposed yet_ — response is `module_id (2B) + sn (4B)` | `LD2420_HAS_SERIAL_NUMBER` |
| `0x0012` | §1.2.7 | Configure system parameters | ✅ | `writeSystemParameter(uint16_t word, uint32_t value)` (low-level), `setSystemMode(uint8_t mode)` (high-level wrapper for `LD2420_SYS_W_MODE`). Single-word only — bulk path waits for a real use case. | `LD2420_HAS_SYSTEM_PARAMS` |
| `0x0013` | §1.2.8 | Read system parameters | ✅ | `readSystemParameter(uint16_t word, uint32_t & out)` (low-level), `getSystemMode(uint8_t & out)` (high-level). | `LD2420_HAS_SYSTEM_PARAMS` |
| `0x0024` | §1.2.9 | Enter factory test mode | ❌ | _not exposed yet_ — ACK payload is the 14-byte block in HLK Table 8 (sub-board model + cascaded chip qty + rx channels + data type + 1DFFT size + chirps per frame + downsampling). Field offsets in `LD2420_FT_OFF_*` | `LD2420_HAS_FACTORY_TEST` |
| `0x0025` | §1.2.10 | Exit factory test mode | ❌ | _not exposed yet_ | `LD2420_HAS_FACTORY_TEST` |
| `0x0026` | §1.2.11 | Send factory test results | ❌ | _not exposed yet_ — primarily a TX register update path per the XLSX note | `LD2420_HAS_FACTORY_TEST` |
| `0x0060 ~ 0x00A0` | §1.2.12 | Custom command range | — | Use directly via the internal command helpers if you need to extend the protocol from sketch space. ACK range is the unusual `0x1060 ~ 0x10A0` (NOT `send_opcode \| 0x0100`) | — |

> **Saturation detection (0x0030).** Listed in HLK Table 4 as a
> firmware-side update; it is not a host command. No driver action needed.

---

## Table 2 — Data path (radar → host)

| Frame | Section | Status | Parser status |
|---|---|:-:|---|
| Energy-report frame (header `F1 F2 F3 F4`, footer `F5 F6 F7 F8`, intra-frame layout from product manual PDF) | §1.1.2 + product manual | 🟡 | `read_frame_()` recognises the envelope and validates the trailer; `parse_data_frame_()` is a stub — discards the frame and updates `radar_uart_last_packet_` only. Full FFT-energy decode lands once the product-manual data-frame layout is transcribed into a dedicated `docs/HLK-LD2420_data_format.md`. |

---

## Table 3 — Capabilities still missing

Priority order, highest-impact first:

1. **`writeAbdParameter` / `readAbdParameter`** (0x0007 / 0x0008) — drives
   the on-chip ABD detection policy. The per-gate threshold packing
   (`gate (2B) | threshold (2B)` inside the 4-byte value of word `0x0012`)
   makes this a non-trivial wrapper.
2. **`parse_data_frame_()` real implementation** — decode the per-gate FFT
   energies into a `uint16_t energies[LD2420_GATE_COUNT]` snapshot field
   plus an atomic snapshot getter.
3. **`requestSerialNumber()`** (0x0011) — quick win.
4. **Factory-test commands** (0x0024 / 0x0025 / 0x0026) — niche but
   completes the V2.2 surface.

---

## Implementation roadmap (status)

| Step | Description | Status |
|---|---|---|
| 1 | `src/ld24xx_common.h` — promote shared envelope + LE helpers out of ld2410_frame.h | ✅ done |
| 2 | `src/ld2420_variants/ld2420_v22.h` — opcode + parameter + feature-flag vocabulary | ✅ done |
| 3 | `src/ld2420_frame.h` — data-path envelope (`F1 F2 F3 F4` / `F5 F6 F7 F8`) | ✅ done |
| 4 | `class ld2420` scaffold — `begin`, `read`, `enterCommandMode`, `exitCommandMode`, `requestFirmwareVersion`, ESP32 autoReadTask + cmd mutex | ✅ done |
| 5 | `examples/ld2420_basicSensor` — smoke-test sketch | ✅ done |
| 6 | Register r/w (0x0001 / 0x0002) | ✅ done |
| 7 | System params (0x0012 / 0x0013) + `setSystemMode` / `getSystemMode` | ✅ done |
| 8 | ESP32 dual-core release-acquire fix in `parse_command_frame_` (member fields are now written before `cmd_ack_seq_` is bumped) | ✅ done |
| 9 | ABD params (0x0007 / 0x0008) — typed setters + per-gate threshold packing | ⏳ next PR |
| 10 | `parse_data_frame_()` real FFT-energy decode | ⏳ blocked on transcribing data-frame layout from product-manual PDF |
| 11 | Serial number (0x0011) | ⏳ |
| 12 | Factory-test mode (0x0024 / 0x0025 / 0x0026) | ⏳ |
| 13 | Hardware validation against a physical LD2420 sample | ⏳ blocked on bench acquisition |
| 14 | Host parser unit test (`tests/test_parser_ld2420/`) mirroring `tests/test_parser*/` for LD2410 | ⏳ |
