/*
 *	An Arduino driver for the Hi-Link LD2420 24 GHz FMCW radar sensor.
 *
 *	The LD2420 is a SIBLING product of the LD2410 family — it shares the
 *	command-frame envelope and LE byte order, but exposes a register-level /
 *	ABD-parameter command vocabulary instead of the LD2410's high-level
 *	"set max gate" + "engineering mode" commands. The data path reports raw
 *	2DFFT energies rather than a classified presence/distance pair, so host
 *	code is responsible for the detection policy.
 *
 *	==========================================================================
 *	STATUS — UNVERIFIED ON HARDWARE
 *
 *	The maintainer's bench does not currently have an LD2420 sample. The
 *	class compiles for esp32 / esp8266 / rp2040 in the CI matrix and the
 *	protocol decoding is transcribed directly from the V2.2 XLSX (see
 *	docs/HLK-LD2420_protocol.md), but every command path and every data
 *	frame is byte-for-byte from the spec, not measured. If you have an
 *	LD2420, please report any discrepancy.
 *	==========================================================================
 *
 *	Released under LGPL-2.1 — see ../LICENSE.
 */
#ifndef ld2420_h
#define ld2420_h
#include <Arduino.h>
#if defined(ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#endif

#include "ld2420_frame.h"
#include "ld2420_variants/ld2420_v22.h"

// ---- Buffer sizing --------------------------------------------------------
// LD2420_MAX_FRAME_LENGTH covers the largest single frame the radar emits.
// The XLSX caps command frames at 64 B total; data frames (FFT energy
// reports) can be larger — the product manual reports a 1024 B tx/rx buffer
// returned by 0x00FF. We size the per-frame buffer at 96 B (enough for any
// command ACK plus a small headroom) and let the circular buffer absorb
// multi-frame bursts. Override before #include if you need bigger.
#ifndef LD2420_MAX_FRAME_LENGTH
#  define LD2420_MAX_FRAME_LENGTH 96
#endif

#ifndef LD2420_BUFFER_SIZE
#  define LD2420_BUFFER_SIZE (4 * LD2420_MAX_FRAME_LENGTH)
#endif

// ---- Debug flags ----------------------------------------------------------
// Symmetric to the LD2410 family — define BEFORE #include <ld2420.h>:
//   LD2420_DEBUG_COMMANDS   log every command sent + ACK received   [ACTIVE]
//   LD2420_DEBUG_DATA       dump every parsed data frame            [reserved]
//   LD2420_DEBUG_PARSE      trace the byte-level parser state       [reserved]
//   LD2420_DEBUG            meta-flag — turns on all three above
#if defined(LD2420_DEBUG)
#  ifndef LD2420_DEBUG_DATA
#    define LD2420_DEBUG_DATA
#  endif
#  ifndef LD2420_DEBUG_COMMANDS
#    define LD2420_DEBUG_COMMANDS
#  endif
#  ifndef LD2420_DEBUG_PARSE
#    define LD2420_DEBUG_PARSE
#  endif
#endif

// Atomic snapshot of the per-frame target state. Use snapshotTargetState()
// to fill this in one critical section when reading from a context that may
// run concurrently with autoReadTask on ESP32 dual-core; calling the
// individual getters separately can observe torn state where presence
// already reflects the new frame but distance / gate energies still hold
// the previous frame's values.
//
// gate_energies is the squared 2DFFT modulus per range gate as decoded
// from the energy frame (see docs/HLK-LD2420_data_format.md). Higher
// values mean more reflected energy at that gate.
struct LD2420TargetState {
    bool     presence;
    uint16_t distance_cm;
    uint16_t gate_energies[16];
};

class ld2420 {

	public:
		// ---- Lifecycle ----------------------------------------------------
		ld2420();
		~ld2420();
		bool begin(Stream & radarStream, bool waitForRadar = true);
		void debug(Stream & terminalStream);
		bool isConnected();
		bool read();

		// ---- Data-frame state accessors -----------------------------------
		// Populated by parse_data_frame_() every time the radar emits an
		// energy frame (45 bytes, header F4 F3 F2 F1 / footer F8 F7 F6 F5;
		// see docs/HLK-LD2420_data_format.md). The radar emits these
		// continuously while outside command mode and its system mode is
		// configured for per-gate energy output.
		//
		// presenceDetected() returns the radar's on-chip ABD classifier
		// decision. The host can also implement its own classifier on top
		// of the raw gate energies if a different policy is wanted.
		//
		// targetDistance() is in centimetres. Valid only when presence is
		// true; the field may hold stale data otherwise.
		//
		// gateEnergy(gate) returns the squared 2DFFT modulus at gate
		// (0..LD2420_GATE_INDEX_LAST). Out-of-range gates return 0.
		//
		// dataFrameReceived() flips true the first time any energy frame
		// is parsed and stays true. Useful to gate "is the radar streaming?"
		// checks separate from "did the last command get an ACK?".
		//
		// These individual getters are NOT atomic against concurrent
		// autoReadTask writes on ESP32 dual-core — use snapshotTargetState()
		// when iterating multiple fields together.
		bool     presenceDetected();
		uint16_t targetDistance();
		uint16_t gateEnergy(uint8_t gate);
		bool     dataFrameReceived();

		// Atomic snapshot of the per-frame target state under
		// portMUX(data_mux_) on ESP32. On non-ESP32 platforms the lock
		// degenerates to a single memcpy-style copy (single-thread
		// assumption — no FreeRTOS scheduler).
		void snapshotTargetState(LD2420TargetState & out) const;

		// ---- Command mode (HLK-2420 §1.2.13 / §1.2.14) --------------------
		// The LD2420 emits waveform data by default; any command issued
		// outside command mode is invalid. The §1.1.1 "usual method" is to
		// send 0x00FF twice (the first call clears any in-flight waveform
		// bytes; the second is the one whose ACK is analysed).
		//
		// enterCommandMode() implements that two-step open with the
		// recommended 100 ms cache-clear delay between sends. On success
		// the radar's reported protocol version and tx/rx buffer size are
		// stored in the matching public fields.
		//
		// exitCommandMode() sends 0x00FE; after success the radar resumes
		// waveform output. Configuration-mutating helpers in this class
		// wrap themselves in enter/exit pairs automatically; call the two
		// directly only when batching multiple commands.
		bool enterCommandMode();
		bool exitCommandMode();
		uint16_t protocol_version    = 0;   // populated by enterCommandMode()
		uint16_t tx_rx_buffer_size   = 0;   // populated by enterCommandMode()

		// ---- Register r/w (HLK-2420 §1.2.2 / §1.2.3) ----------------------
		// Direct read / write of the radar's chip registers. The protocol
		// supports bulk operations (N registers per command), but this API
		// exposes only the single-register path — bulk can be added later
		// once a real use case lands.
		//
		// 0x0001 send payload: `chip_addr (2B) + reg_addr (2B) + value (2B)`.
		// 0x0002 send payload: `chip_addr (2B) + reg_addr (2B)`; ACK result
		// data is the 2-byte register value (LE).
		//
		// Wraps its own enter/exitCommandMode pair, so the caller does not
		// need to be in command mode beforehand.
		bool writeRegister(uint16_t chip, uint16_t reg, uint16_t value);
		bool readRegister (uint16_t chip, uint16_t reg, uint16_t & out);

		// ---- ABD parameters (HLK-2420 §1.2.4 / §1.2.5) --------------------
		// "ABD" = Adaptive Background Detection — the per-gate threshold
		// system the LD2420 uses to classify FFT energies into a presence
		// decision. There are TWO independent threshold blocks (high and
		// low) each with its own active/inactive frame counts and per-gate
		// threshold values; plus three global parameters (roiMin, roiMax,
		// delayTime) used by both blocks.
		//
		// Note on the protocol's asymmetric vocabulary: the WRITE words
		// (LD2420_ABD_W_*) treat roiMin/roiMax/delayTime as global single
		// parameters, but the READ words (LD2420_ABD_R_*) report them
		// separately per high/low block — so reading them back yields two
		// values where a single write only set one. Both vocabularies are
		// exposed verbatim by the low-level API below; the convenience
		// setters use the global write words.
		//
		// 0x0007 send payload: `(write_word (2B) + value (4B)) × N`.
		// 0x0008 send payload: `(read_word (2B)) × N`; ACK result data is
		// the 4-byte value per requested word (LE).
		//
		// Single-word path for both. Each wraps its own enter/exit cfg pair.
		bool writeAbdParameter(uint16_t word, uint32_t value);
		bool readAbdParameter (uint16_t word, uint32_t & out);

		// Convenience — global ROI: writes 0x0000 (roiMin) and 0x0001
		// (roiMax) in a single 0x0007 frame (intra = 14 bytes). Single
		// command-mode roundtrip per call.
		bool setAbdRoi(uint16_t min_gate, uint16_t max_gate);

		// Convenience — per-gate threshold setters. The radar's 0x0012
		// (high) and 0x0022 (low) write words use a packed 4-byte value:
		//   bits [15:0]   = gate index (0..15)
		//   bits [31:16]  = threshold value (squared 2DFFT modulus)
		// e.g. the §1.2.4 example for gate 5 threshold 0x1234 is sent as
		// the four bytes `05 00 34 12` — i.e. uint32_t = 0x12340005.
		// These helpers do the packing for you.
		bool setAbdHighThresholdAtGate(uint16_t gate, uint16_t threshold);
		bool setAbdLowThresholdAtGate (uint16_t gate, uint16_t threshold);

		// Convenience — per-gate threshold readback. Uses the per-gate
		// READ words at LD2420_ABD_R_HIGH_THRESH_BASE + gate (0x0020..0x002F)
		// and LD2420_ABD_R_LOW_THRESH_BASE + gate (0x0030..0x003F). The
		// 4-byte value returned by the radar is the threshold only (no
		// gate-index packing on the read side per Table 6); helpers narrow
		// to uint16_t for the caller.
		bool readAbdHighThresholdAtGate(uint16_t gate, uint16_t & out);
		bool readAbdLowThresholdAtGate (uint16_t gate, uint16_t & out);

		// ---- System parameters (HLK-2420 §1.2.7 / §1.2.8) -----------------
		// Generic read / write of the system parameter words enumerated in
		// LD2420_SYS_W_*. Each parameter is a 4-byte LE value.
		//
		// 0x0012 send payload: `(word (2B) + value (4B)) × N`.
		// 0x0013 send payload: `(word (2B)) × N`; ACK result data is the
		// 4-byte value per requested word (LE).
		//
		// Single-word path only — the bulk path waits for a real use case.
		// Both methods wrap their own enter/exitCommandMode pair.
		bool writeSystemParameter(uint16_t word, uint32_t value);
		bool readSystemParameter (uint16_t word, uint32_t & out);

		// Convenience wrappers around the most useful system-parameter word
		// (LD2420_SYS_W_MODE). Mode values: LD2420_SYS_MODE_TRANSPARENT (0),
		// LD2420_SYS_MODE_MTT (1), LD2420_SYS_MODE_VS (2), LD2420_SYS_MODE_GR
		// (3), or LD2420_SYS_MODE_CUSTOM_BASE+ (≥10).
		bool setSystemMode(uint8_t mode);
		bool getSystemMode(uint8_t & out);

		// ---- Serial number (HLK-2420 §1.2.6) ------------------------------
		// 0x0011 — read the module identifier + serial number. ACK payload
		// is 6 bytes: `module_id (2B LE) + serial_number (4B LE)`. On
		// success populates `module_identification` and `serial_number`.
		// Wraps its own enter/exitCommandMode pair.
		bool requestSerialNumber();
		uint16_t module_identification = 0;
		uint32_t serial_number         = 0;

		// ---- Factory test mode (HLK-2420 §1.2.9 / §1.2.10 / §1.2.11) ------
		// 0x0024 — enter factory test. ACK payload is 14 bytes of radar
		// metadata (Table 8): sub-board model, cascaded chip count, receive
		// channels, data-type discriminator (1DFFT / 2DFFT / 2DFFT-peak /
		// DSRAW — see LD2420_FT_DATA_TYPE_*), 1DFFT size, chirps per frame,
		// downsampling interval. On success populates the `ft_*` fields.
		//
		// 0x0025 — exit factory test. No response payload.
		//
		// 0x0026 — send factory test results. Takes a single (address,
		// data) pair; per the XLSX this command "mainly updates the TX
		// register". No response payload.
		//
		// All three wrap their own enter/exitCommandMode pair. Factory-test
		// mode is orthogonal to command mode in the protocol — every command
		// is gated on command mode being open; what makes a command part of
		// the "factory test" set is whether the radar firmware needs the
		// test-mode flag (set by 0x0024) before honouring it.
		bool enterFactoryTestMode();
		bool exitFactoryTestMode();
		bool sendFactoryTestResult(uint16_t address, uint16_t data);

		uint16_t ft_subboard_model      = 0;   // reserved by HLK, fill 0
		uint16_t ft_cascaded_chips      = 0;   // 1 = single, 2 = dual, …
		uint16_t ft_rx_channels         = 0;
		uint16_t ft_data_type           = 0;   // LD2420_FT_DATA_TYPE_*
		uint16_t ft_1dfft_size          = 0;
		uint16_t ft_chirps_per_frame    = 0;
		uint16_t ft_downsampling        = 0;   // 1 = no downsampling

		// ---- Firmware version (HLK-2420 §1.2.1) ---------------------------
		// 0x0000 — read version. The ACK payload is a 2-byte LE length field
		// followed by an ASCII version string (e.g. "v1.4.14" — 7 bytes).
		//
		// The version string is copied verbatim into firmware_version_ascii[]
		// and null-terminated; firmware_version_length is the byte count
		// reported by the radar (without the NUL). The caller-friendly
		// numeric breakdown (major / minor / patch) is best-effort parsed
		// from the ASCII for compatibility with the LD2410 family API.
		bool requestFirmwareVersion();
		char     firmware_version_ascii[16] = {0};   // NUL-terminated copy
		uint16_t firmware_version_length    = 0;     // bytes reported (no NUL)
		uint16_t firmware_major_version     = 0;     // parsed from ASCII
		uint16_t firmware_minor_version     = 0;
		uint16_t firmware_patch_version     = 0;

		// ---- Snapshot + auto-read task ------------------------------------
#if defined(ESP32)
		bool autoReadTask(uint32_t stack = 4096, UBaseType_t priority = 1, BaseType_t core = tskNO_AFFINITY);
		void stopAutoReadTask();
		bool isAutoReadTaskRunning();
#endif

		// ---- Command serialization (concurrency safety) -------------------
		// RAII guard around lock_command_ / unlock_command_; same pattern as
		// class ld2410. On non-ESP32 platforms the lock degenerates to a
		// single-threaded no-op.
		class CommandTransaction {
			ld2420 & sensor_;
			bool acquired_;
		public:
			explicit CommandTransaction(ld2420 & s, uint32_t timeout_ms = 1000)
				: sensor_(s), acquired_(s.lock_command_(timeout_ms)) {}
			~CommandTransaction() { if (acquired_) sensor_.unlock_command_(); }
			bool ok() const { return acquired_; }
			CommandTransaction(const CommandTransaction&) = delete;
			CommandTransaction& operator=(const CommandTransaction&) = delete;
		};

	private:
		Stream * radar_uart_ = nullptr;
		Stream * debug_uart_ = nullptr;
		uint32_t radar_uart_timeout         = 100;
		uint32_t radar_uart_last_packet_    = 0;
		uint32_t radar_uart_command_timeout_ = 200;   // LD2420 ACKs are larger than LD2410's; bump a bit

		// Command-channel state — mirrors the LD2410 family.
		uint16_t latest_ack_           = 0;          // 2-byte ACK opcode (LE word)
		bool     latest_command_success_ = false;
		uint8_t  cmd_seq_              = 0;          // bumped before each issue
		uint8_t  cmd_ack_seq_          = 0;          // mirrored by parser on match
		uint16_t expected_ack_opcode_  = 0;          // set by command issuer
		bool     waiting_for_ack_      = false;
		bool     ack_frame_            = false;      // current frame is a cmd ACK
		uint16_t last_valid_frame_length = 0;

		uint8_t radar_data_frame_[LD2420_MAX_FRAME_LENGTH];
		uint16_t radar_data_frame_position_ = 0;

		// Scratch state used by single-shot read paths. The parser stashes
		// the most recent ACK payload here; the public read methods consume
		// it under the same cmd_seq_/cmd_ack_seq_ release-acquire pattern
		// as the rest of the command state, so reads on ESP32 dual-core do
		// not see torn values.
		uint16_t last_register_value_     = 0;   // 0x0102 ACK payload
		uint32_t last_system_param_value_ = 0;   // 0x0113 ACK payload
		uint32_t last_abd_param_value_    = 0;   // 0x0108 ACK payload

		// Data-frame state — written by parse_data_frame_(), read by the
		// public getters and snapshotTargetState(). Guarded by data_mux_ on
		// ESP32 so getters do not see torn writes from autoReadTask.
		bool     presence_detected_       = false;
		uint16_t target_distance_cm_      = 0;
		uint16_t gate_energies_[LD2420_GATE_COUNT] = {};
		bool     data_frame_received_     = false;

		uint8_t  circular_buffer[LD2420_BUFFER_SIZE];
		uint16_t buffer_head = 0;
		uint16_t buffer_tail = 0;

#if defined(ESP32)
		TaskHandle_t taskHandle_ = nullptr;
		mutable portMUX_TYPE data_mux_ = portMUX_INITIALIZER_UNLOCKED;
		SemaphoreHandle_t cmd_mutex_ = nullptr;
		static void taskFunction(void * param);
#endif

		// ---- Parser plumbing ---------------------------------------------
		void add_to_buffer(uint8_t byte);
		bool read_from_buffer(uint8_t & byte);
		bool check_frame_start_();
		bool check_frame_end_();
		bool read_frame_();
		bool parse_command_frame_();
		bool parse_data_frame_();           // stub — fills in when host-side
		                                    // FFT decode lands in a later PR

		// ---- Command issuing ---------------------------------------------
		// LD2420 opcodes are real 2-byte LE values (vs LD2410's 1-byte + 0x00
		// implicit high byte), so begin_command_, expected_ack_opcode_ and
		// wait_for_ack_ all carry uint16_t — not uint8_t.
		void begin_command_(uint16_t expected_op);
		void send_command_preamble_();
		void send_command_postamble_();
		bool wait_for_ack_(uint16_t expected_op, uint32_t timeout_ms);

		// Empty-payload command: cmd-word only, intra=2. Used by 0x0000
		// (read version), 0x0011 (read SN), 0x00FE (end cmd mode), 0x0024
		// (enter factory test), 0x0025 (exit factory test), 0x0026 (send
		// factory test results).
		void send_simple_command_(uint16_t opcode);

		bool lock_command_(uint32_t timeout_ms);
		void unlock_command_();

		// Internal helper consumed by enterCommandMode() — sends one 0x00FF
		// with the 2-byte upper-computer version payload (we always send 1).
		// The §1.1.1 "double tap" lives in enterCommandMode() itself so the
		// public method's docstring matches its behaviour exactly.
		bool send_enable_cfg_();

		// Parser helpers for ASCII version-string handling.
		void store_firmware_version_(const uint8_t * payload, uint16_t len);

		void print_frame_();
};

#endif
