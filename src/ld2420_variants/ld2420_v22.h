/*
 *	LD2420 — opcode, parameter and feature definitions (protocol V2.2).
 *
 *	Reference: docs/HLK-LD2420_protocol.md (Hi-Link XLSX V2.2, 2023-04-18,
 *	protocol version reported by 0x00FF = 2).
 *
 *	==========================================================================
 *	STATUS — UNVERIFIED ON HARDWARE
 *
 *	Every value in this header is transcribed directly from the Hi-Link XLSX
 *	V2.2 protocol document. NONE of these opcodes, parameter encodings, frame
 *	layouts or feature flags has been exercised against a real LD2420 unit by
 *	the maintainers of this fork — we do not currently have an LD2420 on the
 *	bench. Treat this file as a faithful translation of the spec, not as
 *	field-validated code.
 *
 *	If you have an LD2420 on your bench: please report any discrepancy
 *	between the spec values here and what the firmware actually accepts —
 *	especially around ABD parameter encodings, the 0x0024 factory-test
 *	response field order, and the per-gate threshold packing for word 0x0012
 *	in the 0x0007 Configure ABD command. PRs welcome.
 *	==========================================================================
 *
 *	The LD2420 is a SIBLING product of the LD2410 family, not a variant of
 *	it. It shares the wire-level command envelope (FD FC FB FA / 04 03 02 01)
 *	and the LE byte order, but the data-frame envelope is byte-reversed
 *	(F1 F2 F3 F4 / F5 F6 F7 F8), opcodes are real 2-byte values (vs the
 *	1-byte + 0x00 high byte on LD2410), and the command vocabulary is
 *	register-level / ABD-parameter-based (vs LD2410's high-level
 *	"set max gate" / "engineering mode" commands).
 *
 *	The radar reports raw FFT energies — there is NO on-chip presence
 *	classifier in the protocol layer. The host implements detection policy.
 *
 *	This header is consumed by src/ld2420.h via #include and defines the
 *	complete vocabulary needed by class ld2420. It is standalone — it does
 *	not include any LD2410 headers.
 */
#ifndef ld2420_v22_h
#define ld2420_v22_h

#include <stdint.h>

// ---- Variant identification -----------------------------------------------
#define LD2420_VARIANT_NAME "LD2420"
#define LD2420_PROTOCOL_VERSION 2

// ---- Distance gates (HLK-2420 §1.2.5 Table 6) -----------------------------
// 16 gates indexed 0..15 (the high/low-threshold blocks at 0x0020..0x002F and
// 0x0030..0x003F enumerate one word per gate). Gate width is NOT specified
// in the V2.2 protocol document; consult the LD2420 product manual PDF for
// the physical bin size if you need to compute absolute distance.
#define LD2420_GATE_COUNT             16
#define LD2420_GATE_INDEX_FIRST       0
#define LD2420_GATE_INDEX_LAST        15

// ---- UART defaults --------------------------------------------------------
// The XLSX does not state a factory baud; the product manual PDF reports
// 115200 8N1 as the default. Adjust here if your unit ships differently.
#define LD2420_DEFAULT_BAUD           115200

// ---- Command envelope sizing ---------------------------------------------
// HLK-2420 §1.1.1: single command + its envelope must fit in 64 bytes total
// (the open-command-mode ACK reports the actual buffer; example shows 1024 B
// on production firmware). The 64 B figure is the conservative cross-firmware
// upper bound the document recommends for splitting long writes.
#define LD2420_MAX_FRAME_LENGTH_BYTES 96

// ---- Frame-format constants ----------------------------------------------
// The intra-frame length field is a 2-byte LE count of the bytes between
// the header and the footer (i.e. excluding the 4-byte head and 4-byte tail).
// HLK-2420 §1.1.2 / Table 1.
#define LD2420_INTRA_HEADER_BYTES     2   // length field
#define LD2420_ENVELOPE_BYTES         8   // 4-head + 2-len + N + 4-tail → 10 minimum

// ---- Command opcodes (HLK-2420 §1.2 Table 3) ------------------------------
// All opcodes are real 2-byte LE values (high byte is significant — e.g.
// 0x00FF means wire bytes "FF 00", and 0x00FE means "FE 00").
//
// ACK opcode = send opcode | 0x0100 (e.g. 0x0000 → 0x0100).
#define LD2420_OP_READ_VERSION         0x0000  // §1.2.1
#define LD2420_OP_WRITE_REGISTER       0x0001  // §1.2.2
#define LD2420_OP_READ_REGISTER        0x0002  // §1.2.3
#define LD2420_OP_CONFIG_ABD_PARAMS    0x0007  // §1.2.4
#define LD2420_OP_READ_ABD_PARAMS      0x0008  // §1.2.5
#define LD2420_OP_READ_SN              0x0011  // §1.2.6
#define LD2420_OP_CONFIG_SYS_PARAMS    0x0012  // §1.2.7
#define LD2420_OP_READ_SYS_PARAMS      0x0013  // §1.2.8
#define LD2420_OP_ENTER_FACTORY_TEST   0x0024  // §1.2.9
#define LD2420_OP_EXIT_FACTORY_TEST    0x0025  // §1.2.10
#define LD2420_OP_SEND_FACTORY_RESULT  0x0026  // §1.2.11
#define LD2420_OP_ENABLE_CFG           0x00FF  // §1.2.13 ("Open command mode")
#define LD2420_OP_END_CFG              0x00FE  // §1.2.14 ("Disable command mode")

// ACK bitmask: send_opcode | LD2420_ACK_MASK == ack_opcode for every command.
#define LD2420_ACK_MASK                0x0100

// Custom command range reserved for user-defined opcodes (HLK-2420 §1.2.12):
//   0x0060 ~ 0x00A0  (ACK range 0x1060 ~ 0x10A0 — note this is the only opcode
//   class where the ACK is NOT obtained by ORing 0x0100).
#define LD2420_OP_CUSTOM_MIN           0x0060
#define LD2420_OP_CUSTOM_MAX           0x00A0
#define LD2420_OP_CUSTOM_ACK_MIN       0x1060
#define LD2420_OP_CUSTOM_ACK_MAX       0x10A0

// ---- 0x0007 / 0x0008 ABD parameter words (HLK-2420 Tables 5 & 6) ----------
// "ABD" = Adaptive Background Detection — the per-gate / per-block threshold
// system the LD2420 uses to classify FFT energies into a presence decision.
//
// Configure (0x0007) words. Each is followed by a 4-byte value.
#define LD2420_ABD_W_ROI_MIN              0x0000  // global min detection gate
#define LD2420_ABD_W_ROI_MAX              0x0001  // global max detection gate
#define LD2420_ABD_W_DELAY_TIME           0x0002  // global delay (s)
#define LD2420_ABD_W_HIGH_ACTIVE_FRAMES   0x0010  // high-thresh: min frames to detect
#define LD2420_ABD_W_HIGH_INACTIVE_FRAMES 0x0011  // high-thresh: min frames to clear
#define LD2420_ABD_W_HIGH_THRESHOLD       0x0012  // high-thresh: gate (2B) | thresh (2B)
#define LD2420_ABD_W_LOW_ACTIVE_FRAMES    0x0020  // low-thresh: min frames to detect
#define LD2420_ABD_W_LOW_INACTIVE_FRAMES  0x0021  // low-thresh: min frames to clear
#define LD2420_ABD_W_LOW_THRESHOLD        0x0022  // low-thresh: gate (2B) | thresh (2B)

// Read-ABD (0x0008) words — note these DIFFER from the Configure words above
// and have a separate per-gate block range. See HLK-2420 §1.2.5 Table 6.
#define LD2420_ABD_R_HIGH_ROI_MIN         0x0000
#define LD2420_ABD_R_HIGH_ROI_MAX         0x0001
#define LD2420_ABD_R_HIGH_ACTIVE_FRAMES   0x0002
#define LD2420_ABD_R_HIGH_INACTIVE_FRAMES 0x0003
#define LD2420_ABD_R_LOW_ROI_MIN          0x0010
#define LD2420_ABD_R_LOW_ROI_MAX          0x0011
#define LD2420_ABD_R_LOW_ACTIVE_FRAMES    0x0012
#define LD2420_ABD_R_LOW_INACTIVE_FRAMES  0x0013
// Per-gate threshold blocks: one word per gate 0..15.
//   0x0020 + gate  →  high-threshold value for that gate
//   0x0030 + gate  →  low-threshold value for that gate
// The XLSX text labels both blocks as "high threshold" — confirmed in the doc
// note that the second block is the low-threshold block by context (see
// docs/HLK-LD2420_protocol.md note under Table 6).
#define LD2420_ABD_R_HIGH_THRESH_BASE     0x0020
#define LD2420_ABD_R_LOW_THRESH_BASE      0x0030

// ---- 0x0012 system parameter words (HLK-2420 Table 7) --------------------
#define LD2420_SYS_W_MODE                 0x0000  // 0=transparent 1=MTT 2=VS 3=GR ≥10 custom
#define LD2420_SYS_W_UPLOAD_SAMPLE_RATE   0x0001  // downsampling ratio (vendor-defined)
#define LD2420_SYS_W_DEBUG_MODE           0x0002  // vendor-internal

// systemMode enumeration values (Table 7 column 3).
#define LD2420_SYS_MODE_TRANSPARENT       0
#define LD2420_SYS_MODE_MTT               1
#define LD2420_SYS_MODE_VS                2
#define LD2420_SYS_MODE_GR                3
#define LD2420_SYS_MODE_CUSTOM_BASE       10  // user-defined modes start at 10

// ---- 0x0024 factory-test response field positions (HLK-2420 Table 8) -----
// The ACK payload after the 2-byte return value is 14 bytes (7 × uint16 LE).
// Offsets are from the first byte AFTER the 4-byte return-value pair.
#define LD2420_FT_OFF_SUBBOARD_MODEL      0   // reserved, fill 0
#define LD2420_FT_OFF_CASCADED_QTY        2   // 1 = single chip, 2 = dual chip
#define LD2420_FT_OFF_RX_CHANNELS         4   // number of receive channels
#define LD2420_FT_OFF_DATA_TYPE           6   // 0=1DFFT 1=2DFFT 2=2DFFT_PEAK 3=DSRAW
#define LD2420_FT_OFF_1DFFT_SIZE          8
#define LD2420_FT_OFF_CHIRPS_PER_FRAME    10
#define LD2420_FT_OFF_DOWNSAMPLING        12  // 1 means no downsampling

// data-type field values
#define LD2420_FT_DATA_TYPE_1DFFT         0
#define LD2420_FT_DATA_TYPE_2DFFT         1
#define LD2420_FT_DATA_TYPE_2DFFT_PEAK    2
#define LD2420_FT_DATA_TYPE_DSRAW         3

// ---- Update content (HLK-2420 Table 4) -----------------------------------
// Newer firmware revisions tracked by V2.2:
//   0x0030 — saturation detection function (firmware-side feature).
#define LD2420_UPDATE_SATURATION_DETECT   0x0030

// ---- Feature flags --------------------------------------------------------
// LD2420_HAS_<feature> mirrors the LD2410 family convention: defined when
// the chip supports the command path, so #ifdef gates in ld2420.cpp keep
// methods only when the underlying firmware can answer them.
//
// LD2420 supports the full protocol vocabulary (no variant fragmentation yet
// — this header is the V2.2 baseline). If Hi-Link ships a stripped-down
// LD2420 derivative later, additional headers under src/ld2420_variants/
// can selectively undef these.
#define LD2420_HAS_CONFIGURATION_MODE
#define LD2420_HAS_READ_VERSION
#define LD2420_HAS_REGISTER_RW
#define LD2420_HAS_ABD_PARAMS
#define LD2420_HAS_SYSTEM_PARAMS
#define LD2420_HAS_SERIAL_NUMBER
#define LD2420_HAS_FACTORY_TEST

#endif
