/*
 *	LD2420 data-path envelope.
 *
 *	Reference: docs/HLK-LD2420_protocol.md §1.1.2 and the LD2420 product
 *	manual PDF (energy-report frames have header F1 F2 F3 F4 and footer
 *	F5 F6 F7 F8 — byte-reversed compared to the LD2410 family's F4 F3 F2 F1
 *	/ F8 F7 F6 F5).
 *
 *	The COMMAND-path envelope and the LE write helpers are shared with the
 *	wider HLK 24 GHz radar family — they live in src/ld24xx_common.h and
 *	this header includes that. Only the data-path constants are LD2420-
 *	specific.
 */
#ifndef ld2420_frame_h
#define ld2420_frame_h

#include "ld24xx_common.h"

// Data path (radar → host) — energy report frames. HLK-2420 product manual:
//   header  F1 F2 F3 F4  (wire byte order — little-endian uint32 0xF4F3F2F1)
//   footer  F5 F6 F7 F8  (wire byte order — little-endian uint32 0xF8F7F6F5)
//
// Note this is the byte-reverse of LD2410's F4 F3 F2 F1 / F8 F7 F6 F5. The
// two chip families intentionally do not share the data envelope so a host
// listening to both can disambiguate on the first four header bytes.
constexpr uint8_t LD2420_DATA_FRAME_HEAD[4] = { 0xF1, 0xF2, 0xF3, 0xF4 };
constexpr uint8_t LD2420_DATA_FRAME_TAIL[4] = { 0xF5, 0xF6, 0xF7, 0xF8 };

#endif
