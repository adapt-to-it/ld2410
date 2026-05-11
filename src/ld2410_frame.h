/*
 *	LD2410-family data-path envelope (LD2410, LD2410C, LD2410S).
 *
 *	The COMMAND-path envelope (FD FC FB FA / 04 03 02 01) and the LE write
 *	helpers are shared across the wider HLK 24 GHz radar family — they live
 *	in src/ld24xx_common.h and are forwarded here under their original
 *	ld2410_* names so existing call sites in ld2410.cpp keep working
 *	unchanged.
 *
 *	The DATA-path envelope (F4 F3 F2 F1 / F8 F7 F6 F5, and the intra-frame
 *	AA…55 bytes from HLK Table 9) is LD2410-specific and stays here. The
 *	LD2420 sibling uses byte-reversed magic (F1 F2 F3 F4 / F5 F6 F7 F8);
 *	see src/ld2420_frame.h.
 *
 *	Including this header is safe regardless of which LD2410_VARIANT_* is
 *	selected — it does not depend on the variant.
 */
#ifndef ld2410_frame_h
#define ld2410_frame_h

#include "ld24xx_common.h"

// Command-path envelope: shared with the rest of the HLK family. Aliases
// for the original ld2410-prefixed names keep ld2410.cpp call sites
// untouched while the canonical names live in ld24xx_common.h.
constexpr const uint8_t (&LD2410_CMD_FRAME_HEAD)[4] = LD24XX_CMD_FRAME_HEAD;
constexpr const uint8_t (&LD2410_CMD_FRAME_TAIL)[4] = LD24XX_CMD_FRAME_TAIL;

inline void ld2410_write_cmd_frame_head(Stream * uart) { ld24xx_write_cmd_frame_head(uart); }
inline void ld2410_write_cmd_frame_tail(Stream * uart) { ld24xx_write_cmd_frame_tail(uart); }
inline void ld2410_write_le16(Stream * uart, uint16_t value) { ld24xx_write_le16(uart, value); }
inline void ld2410_write_le32(Stream * uart, uint32_t value) { ld24xx_write_le32(uart, value); }

// Data path (radar → host) — standard / engineering frames: HLK Tables 8/9.
// Both base/C and S use these 4-byte magic sequences for non-minimal frames.
constexpr uint8_t LD2410_DATA_FRAME_HEAD[4] = { 0xF4, 0xF3, 0xF2, 0xF1 };
constexpr uint8_t LD2410_DATA_FRAME_TAIL[4] = { 0xF8, 0xF7, 0xF6, 0xF5 };

// Intra-frame head/tail bytes inside a non-minimal data frame: HLK Table 9.
constexpr uint8_t LD2410_DATA_INTRA_HEAD = 0xAA;
constexpr uint8_t LD2410_DATA_INTRA_TAIL = 0x55;
constexpr uint8_t LD2410_DATA_INTRA_CHECK = 0x00;

#endif
