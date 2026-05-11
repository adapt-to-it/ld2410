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

// Data path (radar → host) — energy report frames.
//
// Wire byte order:
//   header  F4 F3 F2 F1   (little-endian uint32 0xF1F2F3F4)
//   footer  F8 F7 F6 F5   (little-endian uint32 0xF5F6F7F8)
//
// IMPORTANT: this is THE SAME on-wire byte order as the LD2410 family
// (cf. LD2410_DATA_FRAME_HEAD / LD2410_DATA_FRAME_TAIL in ld2410_frame.h).
// Earlier drafts of this header AND the docs claimed the LD2420 envelope
// was byte-reversed; that was a misreading of ESPHome's `ENERGY_FRAME_HEADER
// = 0xF1F2F3F4` constant, which produces F4 F3 F2 F1 on the wire when stored
// little-endian — not F1 F2 F3 F4. The actual wire bytes match LD2410.
//
// Provenance: cross-checked against ESPHome's production `ld2420` component
// (esphome/esphome) which has been validated against real LD2420 hardware
// for years. The Hi-Link V2.2 XLSX (docs/HLK-LD2420_protocol.md) covers
// only the command side; the data-frame layout lives in the LD2420 product
// manual PDF which is not redistributed in this repository.
//
// See docs/HLK-LD2420_data_format.md for the full intra-frame layout.
constexpr uint8_t LD2420_DATA_FRAME_HEAD[4] = { 0xF4, 0xF3, 0xF2, 0xF1 };
constexpr uint8_t LD2420_DATA_FRAME_TAIL[4] = { 0xF8, 0xF7, 0xF6, 0xF5 };

#endif
