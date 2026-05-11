/*
 *	Helpers shared across the HLK 24 GHz radar family (LD2410, LD2410C,
 *	LD2410S, LD2420 — and any future sibling that reuses the same
 *	command-frame envelope).
 *
 *	What lives here:
 *	  - The command-path frame magic (FD FC FB FA … 04 03 02 01) — identical
 *	    on every chip in the family per the HLK datasheets.
 *	  - Little-endian write helpers for 16-/32-bit fields — every chip
 *	    transmits length, command word and parameter values LE.
 *
 *	What does NOT live here:
 *	  - Data-path envelopes. LD2410 family uses F4 F3 F2 F1 / F8 F7 F6 F5;
 *	    LD2420 uses the byte-reversed F1 F2 F3 F4 / F5 F6 F7 F8. Each chip's
 *	    frame header (ld2410_frame.h, ld2420_frame.h) defines its own.
 *	  - Opcodes / parameter words / feature flags. Each chip's variant
 *	    header under src/ld2410_variants/ or src/ld2420_variants/ owns these.
 *
 *	Naming convention: ld24xx_* (lowercase prefix) for everything generic.
 *	Chip-specific symbols use the chip prefix (ld2410_*, ld2420_*).
 *
 *	Header-only, safe to include from multiple TUs.
 */
#ifndef ld24xx_common_h
#define ld24xx_common_h

#include <Arduino.h>
#include <stdint.h>

// Command path (host → radar): same magic bytes for every HLK 24 GHz radar.
// LD2410 datasheets call these out in §2.1 Tabella 2/4; LD2420 §1.1.2 Table 1.
constexpr uint8_t LD24XX_CMD_FRAME_HEAD[4] = { 0xFD, 0xFC, 0xFB, 0xFA };
constexpr uint8_t LD24XX_CMD_FRAME_TAIL[4] = { 0x04, 0x03, 0x02, 0x01 };

// Emit the 4-byte command-frame head/tail to a Stream. Centralised so that
// every chip's send_command_preamble_/postamble_ collapses to one call.
inline void ld24xx_write_cmd_frame_head(Stream * uart) {
	uart->write(LD24XX_CMD_FRAME_HEAD[0]);
	uart->write(LD24XX_CMD_FRAME_HEAD[1]);
	uart->write(LD24XX_CMD_FRAME_HEAD[2]);
	uart->write(LD24XX_CMD_FRAME_HEAD[3]);
}

inline void ld24xx_write_cmd_frame_tail(Stream * uart) {
	uart->write(LD24XX_CMD_FRAME_TAIL[0]);
	uart->write(LD24XX_CMD_FRAME_TAIL[1]);
	uart->write(LD24XX_CMD_FRAME_TAIL[2]);
	uart->write(LD24XX_CMD_FRAME_TAIL[3]);
}

// Emit a little-endian 16-bit value. The protocol uses LE consistently for
// length fields, command words and parameter values; this lets call sites
// read at protocol level (one 16-bit param) instead of two raw bytes.
inline void ld24xx_write_le16(Stream * uart, uint16_t value) {
	uart->write((uint8_t)(value & 0xFF));
	uart->write((uint8_t)((value >> 8) & 0xFF));
}

inline void ld24xx_write_le32(Stream * uart, uint32_t value) {
	uart->write((uint8_t)(value & 0xFF));
	uart->write((uint8_t)((value >> 8) & 0xFF));
	uart->write((uint8_t)((value >> 16) & 0xFF));
	uart->write((uint8_t)((value >> 24) & 0xFF));
}

#endif
