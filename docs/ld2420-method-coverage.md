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
| `0x0007` | §1.2.4 | Configure ABD parameters | ✅ | Low-level: `writeAbdParameter(word, value)`. Convenience: `setAbdRoi(min_gate, max_gate)` (writes 0x0000 + 0x0001 in one frame), `setAbdHighThresholdAtGate(gate, threshold)` / `setAbdLowThresholdAtGate(gate, threshold)` (handle the `gate (low16) \| threshold (high16)` packing for write words 0x0012 / 0x0022). | `LD2420_HAS_ABD_PARAMS` |
| `0x0008` | §1.2.5 | Read ABD parameters | ✅ | Low-level: `readAbdParameter(word, &out)`. Convenience: `readAbdHighThresholdAtGate(gate, &out)` / `readAbdLowThresholdAtGate(gate, &out)` for the per-gate blocks at `LD2420_ABD_R_HIGH_THRESH_BASE` (0x0020 + gate) and `LD2420_ABD_R_LOW_THRESH_BASE` (0x0030 + gate). | `LD2420_HAS_ABD_PARAMS` |
| `0x0011` | §1.2.6 | Read serial number | ✅ | `requestSerialNumber()` → populates `module_identification` (uint16_t) and `serial_number` (uint32_t) | `LD2420_HAS_SERIAL_NUMBER` |
| `0x0012` | §1.2.7 | Configure system parameters | ✅ | `writeSystemParameter(uint16_t word, uint32_t value)` (low-level), `setSystemMode(uint8_t mode)` (high-level wrapper for `LD2420_SYS_W_MODE`). Single-word only — bulk path waits for a real use case. | `LD2420_HAS_SYSTEM_PARAMS` |
| `0x0013` | §1.2.8 | Read system parameters | ✅ | `readSystemParameter(uint16_t word, uint32_t & out)` (low-level), `getSystemMode(uint8_t & out)` (high-level). | `LD2420_HAS_SYSTEM_PARAMS` |
| `0x0024` | §1.2.9 | Enter factory test mode | ✅ | `enterFactoryTestMode()` → populates `ft_subboard_model`, `ft_cascaded_chips`, `ft_rx_channels`, `ft_data_type` (LD2420_FT_DATA_TYPE_*), `ft_1dfft_size`, `ft_chirps_per_frame`, `ft_downsampling`. Field offsets in `LD2420_FT_OFF_*`. | `LD2420_HAS_FACTORY_TEST` |
| `0x0025` | §1.2.10 | Exit factory test mode | ✅ | `exitFactoryTestMode()` | `LD2420_HAS_FACTORY_TEST` |
| `0x0026` | §1.2.11 | Send factory test results | ✅ | `sendFactoryTestResult(address, data)` — single `(address, data)` pair; per the XLSX this mainly updates the TX register. | `LD2420_HAS_FACTORY_TEST` |
| `0x0060 ~ 0x00A0` | §1.2.12 | Custom command range | — | Use directly via the internal command helpers if you need to extend the protocol from sketch space. ACK range is the unusual `0x1060 ~ 0x10A0` (NOT `send_opcode \| 0x0100`) | — |

> **Saturation detection (0x0030).** Listed in HLK Table 4 as a
> firmware-side update; it is not a host command. No driver action needed.

---

## Table 2 — Data path (radar → host)

| Frame | Section | Status | Parser status |
|---|---|:-:|---|
| Energy-report frame (header `F4 F3 F2 F1`, footer `F8 F7 F6 F5`, 45 bytes total) | not in V2.2 XLSX — see [`docs/HLK-LD2420_data_format.md`](HLK-LD2420_data_format.md) (cross-checked against ESPHome's `ld2420` component) | ✅ | `parse_data_frame_()` decodes the 1-byte presence flag at offset 6, the LE16 distance (cm) at offsets 7..8, and the 16 × LE16 per-gate energies at offsets 9..40. Exposed via `presenceDetected()` / `targetDistance()` / `gateEnergy(gate)` and the atomic `snapshotTargetState(LD2420TargetState &)`. The on-wire envelope is the SAME as the LD2410 family (`F4 F3 F2 F1` / `F8 F7 F6 F5`) — earlier docs claimed it was byte-reversed, which was wrong. |

---

## Table 3 — Capabilities still missing

Priority order, highest-impact first:

With the serial-number and factory-test commands landed in commit
following `6a20c12`, **every V2.2 XLSX command opcode is now exposed**.
Remaining work is validation and convenience:

1. **Non-energy data-frame modes** — the energy frame (45 B) is decoded;
   the product manual additionally documents 1DFFT / 2DFFT / 2DFFT-peak /
   DSRAW outputs selectable via the factory-test data-type field
   (`LD2420_FT_DATA_TYPE_*`). Adding them needs sample frames captured
   against running silicon for each data type.
2. **Bulk variants** of register r/w, system params, ABD params (multiple
   entries per frame) — not blocking any concrete use case, but completes
   the wire-protocol surface for callers who want fewer round-trips.
3. **Host parser unit test** under `tests/test_parser_ld2420/` — mirror
   the LD2410-family test harness so the parser is validated without
   needing an LD2420 on the bench.
4. **Hardware validation** against a physical LD2420 — blocked on bench
   acquisition.

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
| 9 | ABD params (0x0007 / 0x0008) — low-level `writeAbdParameter` / `readAbdParameter` + convenience `setAbdRoi`, per-gate threshold setters/getters with the `gate \| threshold` packing helper | ✅ done |
| 10 | `parse_data_frame_()` real FFT-energy decode — 45-byte energy frame (presence + distance + 16 × LE16 per-gate energies); state exposed via `presenceDetected` / `targetDistance` / `gateEnergy` / atomic `snapshotTargetState`. Also fixed the byte-order misreading in `ld2420_frame.h`: on-wire data envelope is `F4 F3 F2 F1` / `F8 F7 F6 F5`, same as LD2410. Provenance in [`docs/HLK-LD2420_data_format.md`](HLK-LD2420_data_format.md). | ✅ done |
| 11 | Serial number (0x0011) — `requestSerialNumber()` populating `module_identification` + `serial_number` | ✅ done |
| 12 | Factory-test mode (0x0024 / 0x0025 / 0x0026) — `enterFactoryTestMode` / `exitFactoryTestMode` / `sendFactoryTestResult(address, data)`; entering the mode populates the seven `ft_*` Table-8 fields | ✅ done |
| 13 | Hardware validation against a physical LD2420 sample | ⏳ blocked on bench acquisition |
| 14 | Host parser unit test (`tests/test_parser_ld2420/`) mirroring `tests/test_parser*/` for LD2410 | ⏳ |
