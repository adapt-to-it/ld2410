# HLK-LD2420
## 24 GHz radar module — serial communication protocol

**Manufacturer:** Shenzhen Hi-Link Electronic Co., Ltd
**Document title (zh-CN):** 串口命令格式 (Serial Port Command Format)
**Version:** V2.2
**Last modified:** 2023-04-18
**Protocol version reported by `Open command mode` (0x00FF):** 2
**Copyright:** © Shenzhen Hi-Link Electronic Co., Ltd

> **Source.** This document is transcribed from the official Hi-Link spreadsheet `HLK-LD2420 command protocol.xlsx` (sheet `串口命令格式V2.2`, created 2019-10-29, last modified by Liangnicky on 2023-04-18). The XLSX is not redistributed in this repository; the canonical community mirror, together with the full HLK-LD2420 product manual PDF, is at [`soubhik-khan/HLK-LD2420`](https://github.com/soubhik-khan/HLK-LD2420). Command tables, opcode values, frame formats and the wire-level examples here are byte-for-byte from the XLSX.

> **Compared to the LD2410.** The LD2420 shares the same wire framing as the LD2410 family (`FD FC FB FA` … `04 03 02 01`, little-endian, 2-byte intra-frame length) but exposes a **register-level / ABD parameter** command vocabulary instead of the LD2410's high-level configuration commands. There is no built-in human-presence classifier in the protocol layer — the LD2420 reports raw FFT energies and you implement the policy on the host. See the LD2420 product manual PDF for the data-output frame format (energy report frames with header `F1 F2 F3 F4` / footer `F5 F6 F7 F8`) and on-host configuration semantics; this document covers only the **command** side, as the XLSX does.

---

## Catalog

1. [Communication protocols](#1-communication-protocols)
   - 1.1 [Protocol format](#11-protocol-format)
     - 1.1.1 [Radar serial port data format description](#111-radar-serial-port-data-format-description)
     - 1.1.2 [Frame format](#112-frame-format)
     - 1.1.3 [Intra-frame data format](#113-intra-frame-data-format)
     - 1.1.4 [Update content](#114-update-content)
   - 1.2 [Send command with ACK](#12-send-command-with-ack)
     - 1.2.1 [Read version](#121-read-version)
     - 1.2.2 [Write register](#122-write-register)
     - 1.2.3 [Read register](#123-read-register)
     - 1.2.4 [Configure ABD parameters](#124-configure-abd-parameters)
     - 1.2.5 [Read ABD parameters](#125-read-abd-parameters)
     - 1.2.6 [Read serial number](#126-read-serial-number)
     - 1.2.7 [Configure system parameters](#127-configure-system-parameters)
     - 1.2.8 [Read system parameters](#128-read-system-parameters)
     - 1.2.9 [Enter factory test mode](#129-enter-factory-test-mode)
     - 1.2.10 [Exit factory test mode](#1210-exit-factory-test-mode)
     - 1.2.11 [Send factory test results](#1211-send-factory-test-results)
     - 1.2.12 [Custom command range](#1212-custom-command-range)
     - 1.2.13 [Open command mode](#1213-open-command-mode)
     - 1.2.14 [Disable command mode](#1214-disable-command-mode)
   - 1.3 [Examples](#13-examples)

---

## Chart Index

- Table 1 — Command protocol frame format
- Table 2 — Intra-frame data format
- Table 3 — Command opcode summary
- Table 4 — Update content (firmware revisions tracked by this document)
- Table 5 — 0x0007 Configure ABD parameter words
- Table 6 — 0x0008 Read ABD parameter words
- Table 7 — 0x0012 System parameter words
- Table 8 — 0x0024 Enter factory test mode response fields
- Table 9 — Wire examples

---

# 1 Communication protocols

## 1.1 Protocol format

### 1.1.1 Radar serial port data format description

1. **Frame.** Each command data is called a frame; the frame consists of 4 parts: frame header, data length in the frame, data in the frame and tail of the frame.
2. **Intra-frame data.** The intra-frame data starts with a command, followed by the data content.

**Notes:**

1. The maximum data length of a single serial-port command does not exceed **64 bytes** (the size is subject to the actual situation, and each platform may be different — when the upper computer sends the open-command-mode command, the result returned by the lower computer includes the buffer size of the command communication). So when reading and writing multiple registers, if it exceeds 64 bytes, it needs to be divided into multiple commands and issued.
2. **Byte order:** little-endian.
3. Because the serial port will output the radar waveform data by default, it is necessary to switch to **command mode** before issuing the command. The usual method is divided into three steps:
   - (a) Send "Open command mode" (because the chip may still output data, the data received by the serial port will contain waveform data, so the returned result will not be analysed).
   - (b) Clear the serial-port cache data (generally delay about 100 ms, make sure the serial-port data is cleared).
   - (c) Send "Open command mode" again, and analyse the returned result.

   After the end of the command mode, send "Disable command mode" to start the waveform data transmission.
4. The user-defined command ID interval is recommended to be placed in the interval **0x0060 ~ 0x00A0**.

### 1.1.2 Frame format

The LD2420 uses **little-endian** format for all serial data; all bytes in the tables below are hexadecimal.

**Table 1 — Command protocol frame format**

| Frame header | Intra-frame data length | Intra-frame data | End of frame |
|--------------|-------------------------|------------------|--------------|
| FD FC FB FA  | 2 bytes                 | See Table 2      | 04 03 02 01  |

Frame header as little-endian uint32: `0xFAFBFCFD`. End of frame as little-endian uint32: `0x01020304`.

For command return frames, the intra-frame data layout is:

> **2-byte return command + 2-byte return value + N-byte result data**

### 1.1.3 Intra-frame data format

**Table 2 — Intra-frame data format**

| Field            | Width                    |
|------------------|--------------------------|
| Command type     | 2 bytes (see Table 3)    |
| Command data     | N bytes                  |

The return frame layout (built by the radar in response to a command) is:

| Field             | Width                                |
|-------------------|--------------------------------------|
| Return command    | 2 bytes (= command \| 0x0100)        |
| Return value      | 2 bytes (0 = success, other = failed)|
| Return result data| N bytes                              |

**Table 3 — Command opcode summary**

| Command                       | Send opcode      | ACK opcode       | Mandatory? |
|-------------------------------|------------------|------------------|------------|
| Read version                  | 0x0000           | 0x0100           | Yes        |
| Write register                | 0x0001           | 0x0101           | Yes        |
| Read register                 | 0x0002           | 0x0102           | Yes        |
| Configure ABD parameters      | 0x0007           | 0x0107           | No         |
| Read ABD parameters           | 0x0008           | 0x0108           | No         |
| Read serial number            | 0x0011           | 0x0111           | No         |
| Configure system parameters   | 0x0012           | 0x0112           | No         |
| Read system parameters        | 0x0013           | 0x0113           | No         |
| Enter factory test mode       | 0x0024           | 0x0124           | No         |
| Exit factory test mode        | 0x0025           | 0x0125           | No         |
| Send factory test results     | 0x0026           | 0x0126           | No         |
| Custom command range          | 0x0060 ~ 0x00A0  | 0x1060 ~ 0x10A0  | —          |
| Open command mode             | 0x00FF           | 0x01FF           | Yes        |
| Disable command mode          | 0x00FE           | 0x01FE           | Yes        |

### 1.1.4 Update content

**Table 4 — Update content**

| Opcode  | Change                              |
|---------|-------------------------------------|
| 0x0030  | Added saturation detection function |

---

## 1.2 Send command with ACK

Each command must be wrapped in the frame format of Table 1 (header `FD FC FB FA` + 2-byte length + command bytes + footer `04 03 02 01`). The ACK uses the same envelope; its first two intra-frame bytes are the **return command** (send opcode bitwise-OR'd with `0x0100`), followed by 2 bytes of **return value** (0 = success, anything else = failure), then any result data.

### 1.2.1 Read version

- **Command word:** `0x0000`
- **Command data:** none
- **Return value:** 2 bytes (0 success / non-zero failure)
- **Result data:** 2-byte version-string length + version information

The version is returned as an ASCII string. For example, with major version `1`, minor version `4`, patch `14`, the string is `v1.4.14`.

### 1.2.2 Write register

- **Command word:** `0x0001`
- **Command data:** `2-byte chip address + (2-byte register address + 2-byte register data) × N`
- **Return value:** 2 bytes (0 success / non-zero failure)
- **Result data:** none

When the host sends data, it first transmits the 2-byte chip address, then every group of 4 bytes (each group: 2-byte register address + 2-byte register data).

### 1.2.3 Read register

- **Command word:** `0x0002`
- **Command data:** `2-byte chip address + (2-byte register address) × N`
- **Return value:** 2 bytes (0 success / non-zero failure)
- **Result data:** `(2-byte register data) × N`

When the host sends data, it transmits the 2-byte chip address, then every 2 bytes is one register address. When the device returns data, every 2 bytes is one register's value (in the same order as the request).

### 1.2.4 Configure ABD parameters

- **Command word:** `0x0007`
- **Command data:** `(2-byte parameter name + 4-byte parameter value) × N`
- **Return value:** 2 bytes (0 success / non-zero failure)
- **Result data:** none

When the host sends data, every group of 6 bytes contains one parameter-name / parameter-value pair.

**Table 5 — 0x0007 Configure ABD parameter words**

| Parameter name        | Word     | Description                                                                                                  |
|-----------------------|----------|--------------------------------------------------------------------------------------------------------------|
| `roiMin`              | 0x0000   | Minimum detection distance gate (global)                                                                     |
| `roiMax`              | 0x0001   | Maximum detection distance gate (global)                                                                     |
| `delayTime`           | 0x0002   | Delay time (global)                                                                                          |
| `activeFrameNum`      | 0x0010   | High-threshold — minimum number of frames detected                                                           |
| `inactiveFrameNum`    | 0x0011   | High-threshold — minimum number of frames for target to disappear                                            |
| `threshold` (high)    | 0x0012   | High-threshold detection threshold (squared 2DFFT modulus). The 4-byte value packs `gate (2B) + threshold (2B)`, e.g. `05 00 34 12` = gate 5, threshold `0x1234`. |
| `activeFrameNum`      | 0x0020   | Low-threshold — minimum number of frames detected                                                            |
| `inactiveFrameNum`    | 0x0021   | Low-threshold — minimum number of frames for target to disappear                                             |
| `threshold` (low)     | 0x0022   | Low-threshold detection threshold (same packing as above)                                                    |

### 1.2.5 Read ABD parameters

- **Command word:** `0x0008`
- **Command data:** `(2-byte parameter name) × N`
- **Return value:** 2 bytes (0 success / non-zero failure)
- **Result data:** `(4-byte parameter value) × N`

When the host sends data, every 2 bytes is one parameter name. When the device returns data, every 4 bytes is the corresponding parameter value.

**Table 6 — 0x0008 Read ABD parameter words**

| Word        | Parameter                                                                                |
|-------------|------------------------------------------------------------------------------------------|
| 0x0000      | High threshold — `roiMin` (minimum detection distance gate)                              |
| 0x0001      | High threshold — `roiMax` (maximum detection distance gate)                              |
| 0x0002      | High threshold — `activeFrameNum`                                                        |
| 0x0003      | High threshold — `inactiveFrameNum`                                                      |
| 0x0010      | Low threshold — `roiMin`                                                                 |
| 0x0011      | Low threshold — `roiMax`                                                                 |
| 0x0012      | Low threshold — `activeFrameNum` (detect target minimum frame number)                    |
| 0x0013      | Low threshold — `inactiveFrameNum`                                                       |
| 0x0020 ~ 0x002F | High threshold for range gates 0 ~ 15 (one word per gate)                            |
| 0x0030 ~ 0x003F | Low threshold for range gates 0 ~ 15 (one word per gate)                             |

> Note: the XLSX text in this section says the low-threshold block is also labelled "high threshold" in the description ("respectively correspond to the high threshold of range gate 0~15") — assumed to be a typo in the source. By context (Table 5 above and the surrounding structure) `0x0020~0x002F` are the high-threshold thresholds and `0x0030~0x003F` are the low-threshold thresholds.

### 1.2.6 Read serial number

- **Command word:** `0x0011`
- **Command data:** none
- **Return value:** 2 bytes (0 success / non-zero failure)
- **Result data:** `2-byte module identification + 4-byte serial number`

### 1.2.7 Configure system parameters

- **Command word:** `0x0012`
- **Command data:** `(2-byte parameter name + 4-byte parameter value) × N`
- **Return value:** 2 bytes (0 success / non-zero failure)
- **Result data:** none

When the host sends data, every group of 6 bytes contains one parameter-name / parameter-value pair.

**Table 7 — 0x0012 System parameter words**

| Parameter name      | Word     | Allowed values                                                                                          |
|---------------------|----------|---------------------------------------------------------------------------------------------------------|
| `systemMode`        | 0x0000   | `0` = transparent · `1` = MTT · `2` = VS · `3` = GR · ≥ 10 (custom formats recommended to start here)   |
| `uploadSampleRate`  | 0x0001   | Downsampling ratio                                                                                      |
| `debugMode`         | 0x0002   | (radar internal — vendor-defined)                                                                       |

### 1.2.8 Read system parameters

- **Command word:** `0x0013`
- **Command data:** `(2-byte parameter name) × N`
- **Return value:** 2 bytes (0 success / non-zero failure)
- **Result data:** `(4-byte parameter value) × N`

Same wire layout as Read ABD parameters (0x0008): host sends 2-byte names, device returns 4-byte values in the same order.

### 1.2.9 Enter factory test mode

- **Command word:** `0x0024`
- **Command data:** none
- **Return value:** 2 bytes (0 success / non-zero failure)
- **Result data:** see Table 8

**Table 8 — 0x0024 Enter factory test mode response fields**

| Field                    | Width    | Meaning                                                                       |
|--------------------------|----------|-------------------------------------------------------------------------------|
| Sub-board model          | 2 bytes  | Reserved, fill in 0                                                           |
| Cascaded chip quantity   | 2 bytes  | `1` = single chip · `2` = dual chip · …                                       |
| Number of channels       | 2 bytes  | Number of receiving channels                                                  |
| Data type                | 2 bytes  | `0` = 1DFFT · `1` = 2DFFT · `2` = 2DFFT PEAK · `3` = DSRAW                    |
| 1DFFT size               | 2 bytes  | —                                                                             |
| Chirps per frame         | 2 bytes  | —                                                                             |
| Downsampling interval    | 2 bytes  | `1` means no downsampling                                                     |

### 1.2.10 Exit factory test mode

- **Command word:** `0x0025`
- **Command data:** none
- **Return value:** 2 bytes (0 success / non-zero failure)
- **Result data:** none

### 1.2.11 Send factory test results

- **Command word:** `0x0026`
- **Command data:** `(2-byte address + 2-byte data) × N`
- **Return value:** 2 bytes (0 success / non-zero failure)
- **Result data:** none

At present, this command mainly updates the TX register.

### 1.2.12 Custom command range

The opcode range **`0x0060 ~ 0x00A0`** (and the corresponding ACK range **`0x1060 ~ 0x10A0`**) is reserved for user-defined commands. Both payload and ACK semantics are application-specific.

### 1.2.13 Open command mode

Any other command issued to the radar must be executed after this command is issued, otherwise it is invalid.

- **Command word:** `0x00FF`
- **Command data:** 2 bytes — upper-computer version (the host's protocol version it expects)
- **Return value:** 2 bytes (0 success / non-zero failure)
- **Result data:** `2-byte protocol version number + 2-byte transceiver buffer size`

The current protocol version number is **2**.

### 1.2.14 Disable command mode

End command mode; the radar resumes its normal waveform-data output. If further commands are needed afterwards, send "Open command mode" again.

- **Command word:** `0x00FE`
- **Command data:** none
- **Return value:** 2 bytes (0 success / non-zero failure)
- **Result data:** none

---

## 1.3 Examples

All examples are taken verbatim from the XLSX. Hex bytes are wire-order (little-endian as transmitted).

**Table 9 — Wire examples**

### Reading version (0x0000)

**Send (12 bytes):**

```
FD FC FB FA 02 00 00 00 04 03 02 01
```

**Receive (23 bytes):**

```
FD FC FB FA 0D 00 00 01 00 00 07 00 76 31 2E 34 2E 31 34 04 03 02 01
```

Decoded result: success (`00 00`), version-string length 7 (`07 00`), ASCII `v1.4.14` (`76 31 2E 34 2E 31 34`).

### Write a single register (0x0001)

**Send (18 bytes):**

```
FD FC FB FA 08 00 01 00 20 00 41 00 04 C8 04 03 02 01
```

Breakdown: chip address `0x0020`, register `0x0041`, data `0xC804`.

**Receive (14 bytes):**

```
FD FC FB FA 04 00 01 01 00 00 04 03 02 01
```

Result: success.

### Write multiple registers (0x0001, two registers)

**Send (22 bytes):**

```
FD FC FB FA 0C 00 01 00 20 00 41 00 04 C8 42 00 03 00 04 03 02 01
```

Breakdown: chip `0x0020`; register `0x0041` ← `0xC804`; register `0x0042` ← `0x0003`.

**Receive (14 bytes):**

```
FD FC FB FA 04 00 01 01 00 00 04 03 02 01
```

> If the total frame length of a single Write Register command would exceed 64 bytes, split the registers across multiple commands.

### Read a single register (0x0002)

**Send (16 bytes):**

```
FD FC FB FA 04 00 02 00 20 00 41 00 04 03 02 01
```

Breakdown: chip `0x0020`, register `0x0041`.

**Receive (16 bytes):**

```
FD FC FB FA 06 00 02 01 00 00 04 C8 04 03 02 01
```

Result: success; register `0x0041` = `0xC804`.

### Read multiple registers (0x0002, two registers)

**Send (18 bytes):**

```
FD FC FB FA 06 00 02 00 20 00 41 00 42 00 04 03 02 01
```

**Receive (18 bytes):**

```
FD FC FB FA 08 00 02 01 00 00 04 C8 03 00 04 03 02 01
```

Result: register `0x0041` = `0xC804`, register `0x0042` = `0x0003`. Same 64-byte split rule applies for large reads.

### Open command mode (0x00FF)

**Send (12 bytes):**

```
FD FC FB FA 04 00 FF 00 01 00 04 03 02 01
```

(Upper-computer version `0x0001`.)

**Receive (14 bytes):**

```
FD FC FB FA 08 00 FF 01 00 00 02 00 00 04 04 03 02 01
```

Result: success, protocol version `2`, transceiver buffer size `0x0400` = 1024 bytes.

### Disable command mode (0x00FE)

**Send (12 bytes):**

```
FD FC FB FA 02 00 FE 00 04 03 02 01
```

**Receive (14 bytes):**

```
FD FC FB FA 04 00 FE 01 00 00 04 03 02 01
```

### Enter factory test mode (0x0024)

**Send (12 bytes):**

```
FD FC FB FA 02 00 24 00 04 03 02 01
```

**Receive (28 bytes):**

```
FD FC FB FA 12 00 24 01 00 00 00 00 02 00 04 00 00 00 40 00 20 00 02 00 04 03 02 01
```

Decoded: success; sub-board model 0; cascaded chips 2; channels 4; data type 0 (1DFFT); 1DFFT size 64; chirps per frame 32; downsampling interval 2.

### Send factory test results (0x0026)

**Send (12 bytes):**

```
FD FC FB FA 02 00 26 00 04 03 02 01
```

**Receive (28 bytes):**

```
FD FC FB FA 12 00 26 01 00 00 00 00 02 00 04 00 00 00 40 00 20 00 02 00 04 03 02 01
```

(Same response layout as the 0x0024 example above — the XLSX gives identical decoded fields.)

---

## Related files

- [`docs/HLK-LD2410C_protocol.md`](HLK-LD2410C_protocol.md) — sister-protocol document for the LD2410C (same wire framing, different command vocabulary)

## External sources & cross-references

- [`soubhik-khan/HLK-LD2420`](https://github.com/soubhik-khan/HLK-LD2420) — community mirror of Hi-Link's developer artefacts (the source XLSX, the product manual PDF V1.0, and Hi-Link's Windows configuration tool)
- ESPHome `ld2420` component — production implementation reusing the same opcodes and frame headers/footers (independent corroboration of every byte in this document): <https://api-docs.esphome.io/ld2420_8cpp_source>
- [`JoaoSandrini/ld2420-radar`](https://github.com/JoaoSandrini/ld2420-radar) — independent translated PDF of the same protocol

---

# 2 Technical support and contact information

**Shenzhen Hi-Link Electronic Co., Ltd**

- **Website:** [www.hlktech.com](http://www.hlktech.com)
