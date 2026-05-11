/*
 *	Implementation for class ld2420.
 *
 *	Scope of this revision: scaffold + open/close command mode + read
 *	firmware version. Register r/w, ABD/system params, factory-test mode and
 *	data-path FFT decode are stubs and land in subsequent PRs. See
 *	docs/ld2420-method-coverage.md for the full roadmap.
 *
 *	Reference: docs/HLK-LD2420_protocol.md (XLSX V2.2).
 *
 *	UNVERIFIED ON HARDWARE — see banner in src/ld2420.h.
 */
#include "ld2420.h"

ld2420::ld2420() {
}

ld2420::~ld2420() {
#if defined(ESP32)
	if (taskHandle_ != nullptr) {
		vTaskDelete(taskHandle_);
		taskHandle_ = nullptr;
	}
	if (cmd_mutex_ != nullptr) {
		vSemaphoreDelete(cmd_mutex_);
		cmd_mutex_ = nullptr;
	}
#endif
}

void ld2420::debug(Stream & terminalStream) {
	debug_uart_ = &terminalStream;
}

bool ld2420::isConnected() {
	if (millis() - radar_uart_last_packet_ < radar_uart_timeout) {
		return true;
	}
	if (read()) {
		return true;
	}
	return false;
}

bool ld2420::begin(Stream & radarStream, bool waitForRadar) {
	radar_uart_ = &radarStream;
#if defined(ESP32)
	if (cmd_mutex_ == nullptr) {
		cmd_mutex_ = xSemaphoreCreateMutex();
	}
#endif

	if (debug_uart_ != nullptr) {
		debug_uart_->println(F("ld2420 started"));
	}

	if (!waitForRadar) {
		return true;
	}

	if (debug_uart_ != nullptr) {
		debug_uart_->print(F("\nLD2420 firmware: "));
	}

	uint32_t start_time = millis();
	bool firmware_received = false;
	while (millis() - start_time < 1000) {
		if (requestFirmwareVersion()) {
			firmware_received = true;
			break;
		}
		yield();
	}

	if (firmware_received) {
		if (debug_uart_ != nullptr) {
			debug_uart_->print(' ');
			debug_uart_->print(firmware_version_ascii);
		}
		return true;
	}

	if (debug_uart_ != nullptr) {
		debug_uart_->print(F("no response"));
	}
	return false;
}

// ---------------------------------------------------------------------------
// Circular buffer

void ld2420::add_to_buffer(uint8_t byte) {
#if defined(ESP32)
	portENTER_CRITICAL(&data_mux_);
#endif
	circular_buffer[buffer_head] = byte;
	buffer_head = (buffer_head + 1) % LD2420_BUFFER_SIZE;
	if (buffer_head == buffer_tail) {
		// Overflow — drop the oldest byte by advancing tail.
		buffer_tail = (buffer_tail + 1) % LD2420_BUFFER_SIZE;
	}
#if defined(ESP32)
	portEXIT_CRITICAL(&data_mux_);
#endif
}

bool ld2420::read_from_buffer(uint8_t & byte) {
#if defined(ESP32)
	portENTER_CRITICAL(&data_mux_);
#endif
	if (buffer_head == buffer_tail) {
#if defined(ESP32)
		portEXIT_CRITICAL(&data_mux_);
#endif
		return false;
	}
	byte = circular_buffer[buffer_tail];
	buffer_tail = (buffer_tail + 1) % LD2420_BUFFER_SIZE;
#if defined(ESP32)
	portEXIT_CRITICAL(&data_mux_);
#endif
	return true;
}

// ---------------------------------------------------------------------------
// Public read() — drain UART, then advance the parser by one byte at a time
// until a frame ends. Returns true if a frame was completed this call.

bool ld2420::read() {
	if (radar_uart_ == nullptr) {
		return false;
	}
	while (radar_uart_->available()) {
		add_to_buffer(radar_uart_->read());
	}
	return read_frame_();
}

// ---------------------------------------------------------------------------
// Frame reader: byte-driven state machine. Dispatches on header magic to
// either the command-ACK path (FD FC FB FA) or the data-frame path
// (F1 F2 F3 F4). Resync on either header byte at any time.

bool ld2420::read_frame_() {
	uint8_t byte_read;

	while (read_from_buffer(byte_read)) {
		if (radar_data_frame_position_ == 0) {
			// Look for either header's first byte.
			if (byte_read == LD24XX_CMD_FRAME_HEAD[0]) {
				ack_frame_ = true;
				radar_data_frame_[radar_data_frame_position_++] = byte_read;
			} else if (byte_read == LD2420_DATA_FRAME_HEAD[0]) {
				ack_frame_ = false;
				radar_data_frame_[radar_data_frame_position_++] = byte_read;
			}
			continue;
		}

		if (radar_data_frame_position_ < 4) {
			const uint8_t expected = ack_frame_
				? LD24XX_CMD_FRAME_HEAD[radar_data_frame_position_]
				: LD2420_DATA_FRAME_HEAD[radar_data_frame_position_];
			if (byte_read != expected) {
				// Header broke — restart, but allow a fresh header byte to
				// begin a new attempt without losing a byte to resync.
				radar_data_frame_position_ = 0;
				if (byte_read == LD24XX_CMD_FRAME_HEAD[0]) {
					ack_frame_ = true;
					radar_data_frame_[radar_data_frame_position_++] = byte_read;
				} else if (byte_read == LD2420_DATA_FRAME_HEAD[0]) {
					ack_frame_ = false;
					radar_data_frame_[radar_data_frame_position_++] = byte_read;
				}
				continue;
			}
			radar_data_frame_[radar_data_frame_position_++] = byte_read;
			continue;
		}

		// Accumulate until we have the 2-byte length field, then compute the
		// total frame size and accumulate up to that.
		radar_data_frame_[radar_data_frame_position_++] = byte_read;

		if (radar_data_frame_position_ < 6) {
			continue;
		}

		const uint16_t intra_len =
			(uint16_t)radar_data_frame_[4] |
			((uint16_t)radar_data_frame_[5] << 8);
		const uint16_t total_len = 4 + 2 + intra_len + 4;

		if (total_len > LD2420_MAX_FRAME_LENGTH) {
			// Frame too big — drop and resync.
			radar_data_frame_position_ = 0;
			continue;
		}

		if (radar_data_frame_position_ < total_len) {
			continue;
		}

		// We have a complete frame; validate the trailer.
		const uint8_t * tail = ack_frame_
			? LD24XX_CMD_FRAME_TAIL
			: LD2420_DATA_FRAME_TAIL;
		const uint16_t tail_off = total_len - 4;
		bool tail_ok = true;
		for (uint8_t i = 0; i < 4; i++) {
			if (radar_data_frame_[tail_off + i] != tail[i]) {
				tail_ok = false;
				break;
			}
		}

		last_valid_frame_length = total_len;
		bool parsed = false;
		if (tail_ok) {
			parsed = ack_frame_ ? parse_command_frame_() : parse_data_frame_();
		}
		radar_data_frame_position_ = 0;
		if (parsed) {
			return true;
		}
	}
	return false;
}

// ---------------------------------------------------------------------------
// Command-ACK parser. Layout per HLK-2420 §1.1.3:
//   [0..3]  FD FC FB FA
//   [4..5]  intra_len  (LE16)
//   [6..7]  ack_opcode (LE16, = send_opcode | 0x0100)
//   [8..9]  return_value (LE16, 0 = success)
//   [10..]  N bytes of result data
//   [end-4] 04 03 02 01

bool ld2420::parse_command_frame_() {
	if (radar_data_frame_position_ < 12) {
		// Smallest valid ACK has 0 result bytes → 12 total.
		return false;
	}
	const uint16_t ack_opcode =
		(uint16_t)radar_data_frame_[6] |
		((uint16_t)radar_data_frame_[7] << 8);
	const uint16_t status =
		(uint16_t)radar_data_frame_[8] |
		((uint16_t)radar_data_frame_[9] << 8);

	radar_uart_last_packet_ = millis();

	// Decode payload BEFORE publishing the ACK match. On ESP32 dual-core with
	// autoReadTask running, wait_for_ack_ uses cmd_ack_seq_ as a "all state
	// for this ACK is written" signal — flipping it before the member-field
	// writes finish would let the caller read torn state. portENTER_CRITICAL
	// below acts as a release barrier; the matching ENTER on the reader side
	// is the acquire.
	const uint16_t send_opcode = ack_opcode & 0x00FF;
	const uint16_t intra_len =
		(uint16_t)radar_data_frame_[4] | ((uint16_t)radar_data_frame_[5] << 8);
	// intra_len covers cmd-word (2) + return-value (2) + result data; the
	// useful result_len is intra_len - 4.
	const uint16_t result_len = intra_len >= 4 ? intra_len - 4 : 0;
	const uint8_t * payload = &radar_data_frame_[10];

	if (status == 0) {
		switch (send_opcode) {
			case LD2420_OP_READ_VERSION:
				if (result_len >= 2) {
					const uint16_t verstr_len =
						(uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
					if (verstr_len + 2 <= result_len) {
						store_firmware_version_(payload + 2, verstr_len);
					}
				}
				break;

			case LD2420_OP_ENABLE_CFG:
				if (result_len >= 4) {
					protocol_version  = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
					tx_rx_buffer_size = (uint16_t)payload[2] | ((uint16_t)payload[3] << 8);
				}
				break;

			case LD2420_OP_READ_REGISTER:
				// Single-register read: 2 bytes of register data (LE). The
				// bulk path (multiple registers in one request) is not yet
				// exposed; when added, the request will need to remember how
				// many registers were asked so this branch can populate an
				// array. For now we stash only the first register.
				if (result_len >= 2) {
					last_register_value_ =
						(uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
				}
				break;

			case LD2420_OP_READ_SYS_PARAMS:
				// Single-parameter read: 4 bytes of parameter value (LE).
				if (result_len >= 4) {
					last_system_param_value_ =
						(uint32_t)payload[0]
						| ((uint32_t)payload[1] << 8)
						| ((uint32_t)payload[2] << 16)
						| ((uint32_t)payload[3] << 24);
				}
				break;

			case LD2420_OP_READ_ABD_PARAMS:
				// Single-word read: 4 bytes of parameter value (LE). Bulk
				// path can stash an array under cmd_seq_ once exposed.
				if (result_len >= 4) {
					last_abd_param_value_ =
						(uint32_t)payload[0]
						| ((uint32_t)payload[1] << 8)
						| ((uint32_t)payload[2] << 16)
						| ((uint32_t)payload[3] << 24);
				}
				break;

			case LD2420_OP_WRITE_REGISTER:
			case LD2420_OP_CONFIG_ABD_PARAMS:
			case LD2420_OP_CONFIG_SYS_PARAMS:
			case LD2420_OP_END_CFG:
			default:
				// Either no payload to decode, or this revision does not yet
				// expose a parser for the opcode. Caller still sees ACK +
				// status via cmd_seq_/cmd_ack_seq_.
				break;
		}
	}

#if defined(ESP32)
	portENTER_CRITICAL(&data_mux_);
#endif
	latest_ack_ = ack_opcode;
	latest_command_success_ = (status == 0);
	if (ack_opcode == (expected_ack_opcode_ | LD2420_ACK_MASK)) {
		cmd_ack_seq_ = cmd_seq_;
	}
#if defined(ESP32)
	portEXIT_CRITICAL(&data_mux_);
#endif

#if defined(LD2420_DEBUG_COMMANDS)
	if (debug_uart_ != nullptr) {
		debug_uart_->print(F("ld2420 ACK opcode 0x"));
		debug_uart_->print(ack_opcode, HEX);
		if (status != 0) {
			debug_uart_->print(F(" status=0x"));
			debug_uart_->println(status, HEX);
		} else {
			debug_uart_->println(F(" ok"));
		}
	}
#endif
	return true;
}

void ld2420::store_firmware_version_(const uint8_t * payload, uint16_t len) {
	const uint16_t copy = len < (sizeof(firmware_version_ascii) - 1)
		? len
		: (sizeof(firmware_version_ascii) - 1);
	for (uint16_t i = 0; i < copy; i++) {
		firmware_version_ascii[i] = (char)payload[i];
	}
	firmware_version_ascii[copy] = '\0';
	firmware_version_length = len;

	// Best-effort numeric breakdown — e.g. "v1.4.14" → 1, 4, 14.
	firmware_major_version = 0;
	firmware_minor_version = 0;
	firmware_patch_version = 0;
	uint8_t field = 0;
	uint16_t acc = 0;
	bool any_digit = false;
	for (uint16_t i = 0; i <= copy; i++) {
		const char c = i < copy ? firmware_version_ascii[i] : '.';
		if (c >= '0' && c <= '9') {
			acc = acc * 10 + (uint16_t)(c - '0');
			any_digit = true;
		} else if (c == '.') {
			if (any_digit) {
				if      (field == 0) firmware_major_version = acc;
				else if (field == 1) firmware_minor_version = acc;
				else if (field == 2) firmware_patch_version = acc;
				field++;
				acc = 0;
				any_digit = false;
				if (field > 2) break;
			}
		}
		// Any other character (including the leading 'v') is skipped.
	}
}

// ---------------------------------------------------------------------------
// Data-frame parser — STUB. The LD2420 emits FFT-energy frames whose payload
// layout is documented in the LD2420 product manual PDF, not in the V2.2
// XLSX this driver currently targets. Returning false here means read()
// reports "no frame this call" for data frames; the host-visible state stays
// unchanged. A later PR will fill this in once we have a transcribed copy of
// the data-frame layout from the product manual.

bool ld2420::parse_data_frame_() {
	radar_uart_last_packet_ = millis();
	return false;
}

// ---------------------------------------------------------------------------
// Command issuing

void ld2420::send_command_preamble_() {
	ld24xx_write_cmd_frame_head(radar_uart_);
}

void ld2420::send_command_postamble_() {
	ld24xx_write_cmd_frame_tail(radar_uart_);
}

void ld2420::begin_command_(uint16_t expected_op) {
	bool task_running = false;
#if defined(ESP32)
	task_running = (taskHandle_ != nullptr);
	portENTER_CRITICAL(&data_mux_);
#endif
	expected_ack_opcode_ = expected_op;
	cmd_seq_++;
	if (!task_running) {
		buffer_tail = buffer_head;
		radar_data_frame_position_ = 0;
	}
#if defined(ESP32)
	portEXIT_CRITICAL(&data_mux_);
#endif
	if (!task_running) {
		while (radar_uart_->available()) {
			radar_uart_->read();
		}
	}
}

void ld2420::send_simple_command_(uint16_t opcode) {
	begin_command_(opcode);
	send_command_preamble_();
	ld24xx_write_le16(radar_uart_, 0x0002);   // intra_len = 2 (cmd-word only)
	ld24xx_write_le16(radar_uart_, opcode);
	send_command_postamble_();
#if defined(LD2420_DEBUG_COMMANDS)
	if (debug_uart_ != nullptr) {
		debug_uart_->print(F("ld2420 -> 0x"));
		debug_uart_->println(opcode, HEX);
	}
#endif
}

bool ld2420::wait_for_ack_(uint16_t expected_op, uint32_t timeout_ms) {
	uint32_t start = millis();
	bool task_running = false;
#if defined(ESP32)
	task_running = (taskHandle_ != nullptr);
#endif
	while (millis() - start < timeout_ms) {
		if (!task_running) {
			while (radar_uart_->available()) {
				add_to_buffer(radar_uart_->read());
			}
			read_frame_();
		}

		bool got_ack = false;
		bool ok = false;
#if defined(ESP32)
		portENTER_CRITICAL(&data_mux_);
#endif
		if (cmd_ack_seq_ == cmd_seq_ && latest_ack_ == (expected_op | LD2420_ACK_MASK)) {
			got_ack = true;
			ok = latest_command_success_;
		}
#if defined(ESP32)
		portEXIT_CRITICAL(&data_mux_);
#endif
		if (got_ack) {
			return ok;
		}

#if defined(ESP32)
		if (task_running) {
			vTaskDelay(pdMS_TO_TICKS(1));
		}
#endif
		yield();
	}
	return false;
}

bool ld2420::lock_command_(uint32_t timeout_ms) {
#if defined(ESP32)
	if (cmd_mutex_ == nullptr) {
		return true;   // begin() hasn't been called; single-thread assumption.
	}
	return xSemaphoreTake(cmd_mutex_, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
#else
	(void)timeout_ms;
	return true;
#endif
}

void ld2420::unlock_command_() {
#if defined(ESP32)
	if (cmd_mutex_ != nullptr) {
		xSemaphoreGive(cmd_mutex_);
	}
#endif
}

// ---------------------------------------------------------------------------
// Configuration mode (§1.2.13 / §1.2.14)

bool ld2420::send_enable_cfg_() {
	begin_command_(LD2420_OP_ENABLE_CFG);
	send_command_preamble_();
	ld24xx_write_le16(radar_uart_, 0x0004);                 // intra_len = 4
	ld24xx_write_le16(radar_uart_, LD2420_OP_ENABLE_CFG);   // cmd-word
	ld24xx_write_le16(radar_uart_, 0x0001);                 // upper-pc version = 1
	send_command_postamble_();
	return wait_for_ack_(LD2420_OP_ENABLE_CFG, radar_uart_command_timeout_);
}

bool ld2420::enterCommandMode() {
	// HLK-2420 §1.1.1: send open-command-mode TWICE — the first call clears
	// in-flight waveform bytes; the second is the one whose ACK we trust.
	send_enable_cfg_();
	delay(100);                                  // recommended cache-clear delay
	while (radar_uart_->available()) {           // drop anything still in flight
		radar_uart_->read();
	}
	return send_enable_cfg_();
}

bool ld2420::exitCommandMode() {
	send_simple_command_(LD2420_OP_END_CFG);
	return wait_for_ack_(LD2420_OP_END_CFG, radar_uart_command_timeout_);
}

// ---------------------------------------------------------------------------
// Register r/w (§1.2.2 / §1.2.3)
//
// Note on the V2.2 XLSX wire examples: the "Read register" send-frame
// examples in §1.3 print the intra-length field two bytes short of the
// actual intra-frame size (e.g. "04 00" while the body is 6 bytes long).
// Every OTHER send example (write register, open command mode, etc.) and
// every receive example uses the consistent convention "len = full intra
// including cmd-word", and that is what the firmware on the wire accepts.
// We follow the consistent convention. See docs/HLK-LD2420_protocol.md
// note under §1.3 for the discrepancy.

bool ld2420::writeRegister(uint16_t chip, uint16_t reg, uint16_t value) {
	CommandTransaction tx(*this);
	if (!tx.ok()) return false;
	if (!enterCommandMode()) {
		delay(50);
		exitCommandMode();
		return false;
	}
	delay(50);

	begin_command_(LD2420_OP_WRITE_REGISTER);
	send_command_preamble_();
	ld24xx_write_le16(radar_uart_, 0x0008);                    // intra_len = 8
	ld24xx_write_le16(radar_uart_, LD2420_OP_WRITE_REGISTER);  // cmd-word
	ld24xx_write_le16(radar_uart_, chip);
	ld24xx_write_le16(radar_uart_, reg);
	ld24xx_write_le16(radar_uart_, value);
	send_command_postamble_();
	const bool ok = wait_for_ack_(LD2420_OP_WRITE_REGISTER, radar_uart_command_timeout_);

	delay(50);
	exitCommandMode();
	return ok;
}

bool ld2420::readRegister(uint16_t chip, uint16_t reg, uint16_t & out) {
	CommandTransaction tx(*this);
	if (!tx.ok()) return false;
	if (!enterCommandMode()) {
		delay(50);
		exitCommandMode();
		return false;
	}
	delay(50);

	begin_command_(LD2420_OP_READ_REGISTER);
	send_command_preamble_();
	ld24xx_write_le16(radar_uart_, 0x0006);                   // intra_len = 6
	ld24xx_write_le16(radar_uart_, LD2420_OP_READ_REGISTER);  // cmd-word
	ld24xx_write_le16(radar_uart_, chip);
	ld24xx_write_le16(radar_uart_, reg);
	send_command_postamble_();
	const bool ok = wait_for_ack_(LD2420_OP_READ_REGISTER, radar_uart_command_timeout_);

	if (ok) {
#if defined(ESP32)
		portENTER_CRITICAL(&data_mux_);
#endif
		out = last_register_value_;
#if defined(ESP32)
		portEXIT_CRITICAL(&data_mux_);
#endif
	}

	delay(50);
	exitCommandMode();
	return ok;
}

// ---------------------------------------------------------------------------
// System parameters (§1.2.7 / §1.2.8)

bool ld2420::writeSystemParameter(uint16_t word, uint32_t value) {
	CommandTransaction tx(*this);
	if (!tx.ok()) return false;
	if (!enterCommandMode()) {
		delay(50);
		exitCommandMode();
		return false;
	}
	delay(50);

	begin_command_(LD2420_OP_CONFIG_SYS_PARAMS);
	send_command_preamble_();
	ld24xx_write_le16(radar_uart_, 0x0008);                       // intra_len = 8
	ld24xx_write_le16(radar_uart_, LD2420_OP_CONFIG_SYS_PARAMS);  // cmd-word
	ld24xx_write_le16(radar_uart_, word);
	ld24xx_write_le32(radar_uart_, value);
	send_command_postamble_();
	const bool ok = wait_for_ack_(LD2420_OP_CONFIG_SYS_PARAMS, radar_uart_command_timeout_);

	delay(50);
	exitCommandMode();
	return ok;
}

bool ld2420::readSystemParameter(uint16_t word, uint32_t & out) {
	CommandTransaction tx(*this);
	if (!tx.ok()) return false;
	if (!enterCommandMode()) {
		delay(50);
		exitCommandMode();
		return false;
	}
	delay(50);

	begin_command_(LD2420_OP_READ_SYS_PARAMS);
	send_command_preamble_();
	ld24xx_write_le16(radar_uart_, 0x0004);                      // intra_len = 4
	ld24xx_write_le16(radar_uart_, LD2420_OP_READ_SYS_PARAMS);   // cmd-word
	ld24xx_write_le16(radar_uart_, word);
	send_command_postamble_();
	const bool ok = wait_for_ack_(LD2420_OP_READ_SYS_PARAMS, radar_uart_command_timeout_);

	if (ok) {
#if defined(ESP32)
		portENTER_CRITICAL(&data_mux_);
#endif
		out = last_system_param_value_;
#if defined(ESP32)
		portEXIT_CRITICAL(&data_mux_);
#endif
	}

	delay(50);
	exitCommandMode();
	return ok;
}

bool ld2420::setSystemMode(uint8_t mode) {
	return writeSystemParameter(LD2420_SYS_W_MODE, (uint32_t)mode);
}

bool ld2420::getSystemMode(uint8_t & out) {
	uint32_t v = 0;
	if (!readSystemParameter(LD2420_SYS_W_MODE, v)) return false;
	out = (uint8_t)(v & 0xFF);
	return true;
}

// ---------------------------------------------------------------------------
// ABD parameters (§1.2.4 / §1.2.5)
//
// Per-gate threshold packing (write words 0x0012 / 0x0022, per §1.2.4):
//   bits [15:0]  = gate index (0..15)
//   bits [31:16] = threshold value
// e.g. the §1.2.4 example for gate 5 / threshold 0x1234 transmits the four
// bytes `05 00 34 12` — i.e. uint32_t = 0x12340005 on the wire (LE).

bool ld2420::writeAbdParameter(uint16_t word, uint32_t value) {
	CommandTransaction tx(*this);
	if (!tx.ok()) return false;
	if (!enterCommandMode()) {
		delay(50);
		exitCommandMode();
		return false;
	}
	delay(50);

	begin_command_(LD2420_OP_CONFIG_ABD_PARAMS);
	send_command_preamble_();
	ld24xx_write_le16(radar_uart_, 0x0008);                       // intra_len = 8
	ld24xx_write_le16(radar_uart_, LD2420_OP_CONFIG_ABD_PARAMS);  // cmd-word
	ld24xx_write_le16(radar_uart_, word);
	ld24xx_write_le32(radar_uart_, value);
	send_command_postamble_();
	const bool ok = wait_for_ack_(LD2420_OP_CONFIG_ABD_PARAMS, radar_uart_command_timeout_);

	delay(50);
	exitCommandMode();
	return ok;
}

bool ld2420::readAbdParameter(uint16_t word, uint32_t & out) {
	CommandTransaction tx(*this);
	if (!tx.ok()) return false;
	if (!enterCommandMode()) {
		delay(50);
		exitCommandMode();
		return false;
	}
	delay(50);

	begin_command_(LD2420_OP_READ_ABD_PARAMS);
	send_command_preamble_();
	ld24xx_write_le16(radar_uart_, 0x0004);                     // intra_len = 4
	ld24xx_write_le16(radar_uart_, LD2420_OP_READ_ABD_PARAMS);  // cmd-word
	ld24xx_write_le16(radar_uart_, word);
	send_command_postamble_();
	const bool ok = wait_for_ack_(LD2420_OP_READ_ABD_PARAMS, radar_uart_command_timeout_);

	if (ok) {
#if defined(ESP32)
		portENTER_CRITICAL(&data_mux_);
#endif
		out = last_abd_param_value_;
#if defined(ESP32)
		portEXIT_CRITICAL(&data_mux_);
#endif
	}

	delay(50);
	exitCommandMode();
	return ok;
}

bool ld2420::setAbdRoi(uint16_t min_gate, uint16_t max_gate) {
	// Sets both LD2420_ABD_W_ROI_MIN (0x0000) and LD2420_ABD_W_ROI_MAX
	// (0x0001) in a single 0x0007 frame. Intra layout:
	//   cmd_word (2) + word_min (2) + value_min (4) + word_max (2) + value_max (4) = 14 B
	CommandTransaction tx(*this);
	if (!tx.ok()) return false;
	if (!enterCommandMode()) {
		delay(50);
		exitCommandMode();
		return false;
	}
	delay(50);

	begin_command_(LD2420_OP_CONFIG_ABD_PARAMS);
	send_command_preamble_();
	ld24xx_write_le16(radar_uart_, 0x000E);                       // intra_len = 14
	ld24xx_write_le16(radar_uart_, LD2420_OP_CONFIG_ABD_PARAMS);
	ld24xx_write_le16(radar_uart_, LD2420_ABD_W_ROI_MIN);
	ld24xx_write_le32(radar_uart_, (uint32_t)min_gate);
	ld24xx_write_le16(radar_uart_, LD2420_ABD_W_ROI_MAX);
	ld24xx_write_le32(radar_uart_, (uint32_t)max_gate);
	send_command_postamble_();
	const bool ok = wait_for_ack_(LD2420_OP_CONFIG_ABD_PARAMS, radar_uart_command_timeout_);

	delay(50);
	exitCommandMode();
	return ok;
}

bool ld2420::setAbdHighThresholdAtGate(uint16_t gate, uint16_t threshold) {
	const uint32_t packed = ((uint32_t)threshold << 16) | (uint32_t)(gate & 0xFFFF);
	return writeAbdParameter(LD2420_ABD_W_HIGH_THRESHOLD, packed);
}

bool ld2420::setAbdLowThresholdAtGate(uint16_t gate, uint16_t threshold) {
	const uint32_t packed = ((uint32_t)threshold << 16) | (uint32_t)(gate & 0xFFFF);
	return writeAbdParameter(LD2420_ABD_W_LOW_THRESHOLD, packed);
}

bool ld2420::readAbdHighThresholdAtGate(uint16_t gate, uint16_t & out) {
	if (gate > LD2420_GATE_INDEX_LAST) return false;
	uint32_t v = 0;
	if (!readAbdParameter((uint16_t)(LD2420_ABD_R_HIGH_THRESH_BASE + gate), v)) return false;
	out = (uint16_t)(v & 0xFFFF);
	return true;
}

bool ld2420::readAbdLowThresholdAtGate(uint16_t gate, uint16_t & out) {
	if (gate > LD2420_GATE_INDEX_LAST) return false;
	uint32_t v = 0;
	if (!readAbdParameter((uint16_t)(LD2420_ABD_R_LOW_THRESH_BASE + gate), v)) return false;
	out = (uint16_t)(v & 0xFFFF);
	return true;
}

// ---------------------------------------------------------------------------
// Read firmware version (§1.2.1)

bool ld2420::requestFirmwareVersion() {
	CommandTransaction tx(*this);
	if (!tx.ok()) {
		return false;
	}
	if (enterCommandMode()) {
		delay(50);
		send_simple_command_(LD2420_OP_READ_VERSION);
		const bool ok = wait_for_ack_(LD2420_OP_READ_VERSION, radar_uart_command_timeout_);
		delay(50);
		exitCommandMode();
		return ok;
	}
	delay(50);
	exitCommandMode();
	return false;
}

// ---------------------------------------------------------------------------
// ESP32 auto-read task — same pattern as ld2410::autoReadTask().

#if defined(ESP32)
void ld2420::taskFunction(void * param) {
	ld2420 * self = static_cast<ld2420 *>(param);
	for (;;) {
		if (self->radar_uart_ != nullptr) {
			while (self->radar_uart_->available()) {
				self->add_to_buffer(self->radar_uart_->read());
			}
			self->read_frame_();
		}
		vTaskDelay(pdMS_TO_TICKS(1));
	}
}

bool ld2420::autoReadTask(uint32_t stack, UBaseType_t priority, BaseType_t core) {
	if (taskHandle_ != nullptr) {
		return true;   // already running
	}
	BaseType_t ok = xTaskCreatePinnedToCore(
		taskFunction, "ld2420", stack, this, priority, &taskHandle_, core);
	return ok == pdPASS && taskHandle_ != nullptr;
}

void ld2420::stopAutoReadTask() {
	if (taskHandle_ != nullptr) {
		vTaskDelete(taskHandle_);
		taskHandle_ = nullptr;
	}
}

bool ld2420::isAutoReadTaskRunning() {
	return taskHandle_ != nullptr;
}
#endif

void ld2420::print_frame_() {
	if (debug_uart_ == nullptr) {
		return;
	}
	debug_uart_->print(F("ld2420 frame ["));
	debug_uart_->print(last_valid_frame_length);
	debug_uart_->print(F("B]:"));
	for (uint16_t i = 0; i < last_valid_frame_length && i < LD2420_MAX_FRAME_LENGTH; i++) {
		debug_uart_->print(' ');
		if (radar_data_frame_[i] < 0x10) debug_uart_->print('0');
		debug_uart_->print(radar_data_frame_[i], HEX);
	}
	debug_uart_->println();
}

bool ld2420::check_frame_start_() {
	if (radar_data_frame_position_ < 4) return false;
	const uint8_t * head = ack_frame_ ? LD24XX_CMD_FRAME_HEAD : LD2420_DATA_FRAME_HEAD;
	for (uint8_t i = 0; i < 4; i++) {
		if (radar_data_frame_[i] != head[i]) return false;
	}
	return true;
}

bool ld2420::check_frame_end_() {
	if (last_valid_frame_length < 4) return false;
	const uint8_t * tail = ack_frame_ ? LD24XX_CMD_FRAME_TAIL : LD2420_DATA_FRAME_TAIL;
	for (uint8_t i = 0; i < 4; i++) {
		if (radar_data_frame_[last_valid_frame_length - 4 + i] != tail[i]) return false;
	}
	return true;
}
