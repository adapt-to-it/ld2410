// Native host-side unit test for class ld2420's parser.
//
// Mirrors tests/test_parser.cpp (which tests class ld2410) but adapted for:
//   - 16-bit command opcodes (the LD2420 protocol uses real two-byte LE
//     command words; the ACK convention is `send_op | 0x0100`)
//   - the LD2420 energy data frame (45 bytes: head + len + presence +
//     distance + 16 × LE16 energies + tail; see docs/HLK-LD2420_data_format.md)
//   - enterCommandMode() sending 0x00FF twice (HLK §1.1.1 "double tap")
//
// Build & run:  bash tests/run.sh   (from the repo root)
//
// On the host ESP32 is not defined, so the FreeRTOS task path and the
// portMUX critical sections compile out. The test exercises only the
// parser + command-issue/wait_for_ack_ machinery, which is what
// src/ld2420.cpp implements on every target.

#include <Arduino.h>
#include <ld2420.h>
#include <cstdio>
#include <cstring>
#include <vector>
#include <cassert>
#include <initializer_list>

// ---------------------------------------------------------------------------
// Mock UART, same shape as test_parser.cpp's:
//   inject(...)          -- bytes available immediately
//   inject_response(...) -- bytes staged, released when the command postamble
//                            (04 03 02 01) is written, modelling the radar
//                            ACKing only after a complete command arrives.

class MockSerial : public Stream {
    std::vector<uint8_t> q_;
    size_t pos_ = 0;
    std::vector<std::vector<uint8_t>> response_queue_;
    uint8_t last4_[4] = {0};
    int write_count_ = 0;
public:
    void inject(std::initializer_list<uint8_t> bytes) {
        q_.insert(q_.end(), bytes.begin(), bytes.end());
    }
    void inject(const std::vector<uint8_t> & bytes) {
        q_.insert(q_.end(), bytes.begin(), bytes.end());
    }
    void inject_response(const std::vector<uint8_t> & bytes) {
        response_queue_.push_back(bytes);
    }
    int available() override { return (int)(q_.size() - pos_); }
    int read() override {
        if (pos_ >= q_.size()) return -1;
        return q_[pos_++];
    }
    size_t write(uint8_t b) override {
        last4_[write_count_ % 4] = b;
        write_count_++;
        if (write_count_ >= 4) {
            uint8_t b0 = last4_[(write_count_ - 4) % 4];
            uint8_t b1 = last4_[(write_count_ - 3) % 4];
            uint8_t b2 = last4_[(write_count_ - 2) % 4];
            uint8_t b3 = last4_[(write_count_ - 1) % 4];
            if (b0 == 0x04 && b1 == 0x03 && b2 == 0x02 && b3 == 0x01) {
                if (!response_queue_.empty()) {
                    const auto & resp = response_queue_.front();
                    q_.insert(q_.end(), resp.begin(), resp.end());
                    response_queue_.erase(response_queue_.begin());
                }
            }
        }
        return 1;
    }
    using Print::write;
    void clear() {
        q_.clear(); pos_ = 0;
        response_queue_.clear();
        write_count_ = 0;
    }
};

static int failures = 0;

#define CHECK(cond) do { \
    if (!(cond)) { \
        std::fprintf(stderr, "FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); \
        failures++; \
    } \
} while (0)

#define CHECK_EQ(a, b) do { \
    auto _a = (a); auto _b = (b); \
    if (!(_a == _b)) { \
        std::fprintf(stderr, "FAIL %s:%d  %s == %s : got %lld vs %lld\n", \
                     __FILE__, __LINE__, #a, #b, (long long)_a, (long long)_b); \
        failures++; \
    } \
} while (0)

static void drain(ld2420 & r, MockSerial & s) {
    while (s.available() > 0) {
        r.read();
    }
}

// ---------------------------------------------------------------------------
// ACK frame helpers — every LD2420 ACK shares the same outer shape:
//   FD FC FB FA | intra_len (LE16) | ack_op = send_op|0x0100 (LE16) | status (LE16) | result_data ... | 04 03 02 01

static std::vector<uint8_t> make_ack(uint16_t send_op,
                                     std::initializer_list<uint8_t> result_data,
                                     uint16_t status = 0) {
    const uint16_t ack_op = send_op | 0x0100;
    // intra_len covers: cmd_word(2) + status(2) + result_data(N)
    const uint16_t intra_len = 4 + (uint16_t)result_data.size();
    std::vector<uint8_t> v = {
        0xFD, 0xFC, 0xFB, 0xFA,
        (uint8_t)(intra_len & 0xFF), (uint8_t)((intra_len >> 8) & 0xFF),
        (uint8_t)(ack_op & 0xFF), (uint8_t)((ack_op >> 8) & 0xFF),
        (uint8_t)(status & 0xFF), (uint8_t)((status >> 8) & 0xFF),
    };
    for (uint8_t b : result_data) v.push_back(b);
    v.push_back(0x04); v.push_back(0x03); v.push_back(0x02); v.push_back(0x01);
    return v;
}

// Shorthand for ACKs with no result data (most "write" / mode-toggle commands).
static std::vector<uint8_t> make_short_ack(uint16_t send_op, uint16_t status = 0) {
    return make_ack(send_op, {}, status);
}

// 0x00FF ACK: protocol_version (LE16) + tx_rx_buffer_size (LE16).
static std::vector<uint8_t> make_open_cmd_mode_ack(uint16_t proto = 2,
                                                   uint16_t bufsize = 0x0400) {
    return make_ack(0x00FF, {
        (uint8_t)(proto & 0xFF), (uint8_t)((proto >> 8) & 0xFF),
        (uint8_t)(bufsize & 0xFF), (uint8_t)((bufsize >> 8) & 0xFF),
    });
}

// Manual builder for variable-length result blobs (avoids the constexpr
// init-list shape required by make_ack above).
static std::vector<uint8_t> make_ack_blob(uint16_t send_op,
                                          const std::vector<uint8_t> & result_data,
                                          uint16_t status = 0) {
    const uint16_t ack_op = send_op | 0x0100;
    const uint16_t intra_len = 4 + (uint16_t)result_data.size();
    std::vector<uint8_t> v = {
        0xFD, 0xFC, 0xFB, 0xFA,
        (uint8_t)(intra_len & 0xFF), (uint8_t)((intra_len >> 8) & 0xFF),
        (uint8_t)(ack_op & 0xFF), (uint8_t)((ack_op >> 8) & 0xFF),
        (uint8_t)(status & 0xFF), (uint8_t)((status >> 8) & 0xFF),
    };
    v.insert(v.end(), result_data.begin(), result_data.end());
    v.push_back(0x04); v.push_back(0x03); v.push_back(0x02); v.push_back(0x01);
    return v;
}

static std::vector<uint8_t> make_firmware_ack(const char * verstr) {
    const uint16_t len = (uint16_t)std::strlen(verstr);
    std::vector<uint8_t> result = {
        (uint8_t)(len & 0xFF), (uint8_t)((len >> 8) & 0xFF),
    };
    for (uint16_t i = 0; i < len; i++) result.push_back((uint8_t)verstr[i]);
    return make_ack_blob(0x0000, result);
}

// 0x0011 ACK: module_id (LE16) + serial_number (LE32).
static std::vector<uint8_t> make_sn_ack(uint16_t module_id, uint32_t sn) {
    return make_ack(0x0011, {
        (uint8_t)(module_id & 0xFF), (uint8_t)((module_id >> 8) & 0xFF),
        (uint8_t)(sn & 0xFF),
        (uint8_t)((sn >> 8) & 0xFF),
        (uint8_t)((sn >> 16) & 0xFF),
        (uint8_t)((sn >> 24) & 0xFF),
    });
}

// 0x0002 ACK: register_value (LE16).
static std::vector<uint8_t> make_register_read_ack(uint16_t value) {
    return make_ack(0x0002, {
        (uint8_t)(value & 0xFF), (uint8_t)((value >> 8) & 0xFF),
    });
}

// 0x0013 ACK: system_param_value (LE32).
static std::vector<uint8_t> make_sys_param_read_ack(uint32_t value) {
    return make_ack(0x0013, {
        (uint8_t)(value & 0xFF),
        (uint8_t)((value >> 8) & 0xFF),
        (uint8_t)((value >> 16) & 0xFF),
        (uint8_t)((value >> 24) & 0xFF),
    });
}

// 0x0008 ACK: abd_param_value (LE32).
static std::vector<uint8_t> make_abd_param_read_ack(uint32_t value) {
    return make_ack(0x0008, {
        (uint8_t)(value & 0xFF),
        (uint8_t)((value >> 8) & 0xFF),
        (uint8_t)((value >> 16) & 0xFF),
        (uint8_t)((value >> 24) & 0xFF),
    });
}

// 0x0024 ACK: 14-byte HLK Table 8 block (7 × LE16 in this order):
//   subboard_model, cascaded_chips, rx_channels, data_type,
//   1dfft_size, chirps_per_frame, downsampling.
static std::vector<uint8_t> make_factory_test_enter_ack(
        uint16_t subboard, uint16_t cascaded, uint16_t rxch,
        uint16_t dtype, uint16_t fft, uint16_t chirps, uint16_t down) {
    auto push16 = [](std::vector<uint8_t> & v, uint16_t x) {
        v.push_back((uint8_t)(x & 0xFF));
        v.push_back((uint8_t)((x >> 8) & 0xFF));
    };
    std::vector<uint8_t> result;
    push16(result, subboard); push16(result, cascaded); push16(result, rxch);
    push16(result, dtype);    push16(result, fft);      push16(result, chirps);
    push16(result, down);
    return make_ack_blob(0x0024, result);
}

// Stage the FOUR ACKs that wrap a single self-contained command call.
// Each public method on class ld2420 does:
//   enterCommandMode() [sends 0x00FF twice] → command → exitCommandMode()
// so the test must inject 2 open-cmd-mode ACKs + 1 command ACK + 1 close ACK.
static void stage_command_envelope(MockSerial & s,
                                   const std::vector<uint8_t> & command_ack) {
    s.inject_response(make_open_cmd_mode_ack());   // 1st (double-tap) open
    s.inject_response(make_open_cmd_mode_ack());   // 2nd open — the one parsed
    s.inject_response(command_ack);
    s.inject_response(make_short_ack(0x00FE));     // close
}

// ---------------------------------------------------------------------------
// Energy data frame builder.

static std::vector<uint8_t> make_energy_frame(bool presence,
                                              uint16_t distance_cm,
                                              const uint16_t energies[16]) {
    std::vector<uint8_t> v = {
        0xF4, 0xF3, 0xF2, 0xF1,    // head
        0x23, 0x00,                // intra_len = 0x0023 = 35
        (uint8_t)(presence ? 0x01 : 0x00),
        (uint8_t)(distance_cm & 0xFF), (uint8_t)((distance_cm >> 8) & 0xFF),
    };
    for (uint8_t g = 0; g < 16; g++) {
        v.push_back((uint8_t)(energies[g] & 0xFF));
        v.push_back((uint8_t)((energies[g] >> 8) & 0xFF));
    }
    v.push_back(0xF8); v.push_back(0xF7); v.push_back(0xF6); v.push_back(0xF5);
    return v;
}

// ===========================================================================
// Data-path tests
// ===========================================================================

static void test_energy_frame_basic() {
    std::printf("test_energy_frame_basic ... ");
    ld2420 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);

    uint16_t energies[16] = {
        100, 200, 300, 400, 500, 600, 700, 800,
        900, 1000, 1100, 1200, 1300, 1400, 1500, 1600
    };
    s.inject(make_energy_frame(/*presence=*/true, /*distance=*/123, energies));
    drain(r, s);

    CHECK(r.dataFrameReceived());
    CHECK(r.presenceDetected());
    CHECK_EQ((int)r.targetDistance(), 123);
    for (uint8_t g = 0; g < 16; g++) {
        CHECK_EQ((int)r.gateEnergy(g), (int)energies[g]);
    }
    // Out-of-range gate index returns 0 by API contract.
    CHECK_EQ((int)r.gateEnergy(16), 0);
    CHECK_EQ((int)r.gateEnergy(255), 0);
    std::printf("ok\n");
}

static void test_energy_frame_no_presence() {
    std::printf("test_energy_frame_no_presence ... ");
    ld2420 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);

    uint16_t energies[16] = {0};
    energies[3] = 5000;   // one peak
    s.inject(make_energy_frame(/*presence=*/false, /*distance=*/250, energies));
    drain(r, s);

    CHECK(r.dataFrameReceived());
    CHECK(!r.presenceDetected());
    // Distance field is still copied even when presence is false; the API
    // contract says it "may hold stale data" but the parser always reads it.
    CHECK_EQ((int)r.targetDistance(), 250);
    CHECK_EQ((int)r.gateEnergy(3), 5000);
    CHECK_EQ((int)r.gateEnergy(0), 0);
    std::printf("ok\n");
}

static void test_energy_frame_atomic_snapshot() {
    std::printf("test_energy_frame_atomic_snapshot ... ");
    ld2420 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);

    uint16_t energies[16] = {
        11, 22, 33, 44, 55, 66, 77, 88,
        99, 110, 121, 132, 143, 154, 165, 176
    };
    s.inject(make_energy_frame(true, 175, energies));
    drain(r, s);

    LD2420TargetState snap;
    r.snapshotTargetState(snap);
    CHECK(snap.presence);
    CHECK_EQ((int)snap.distance_cm, 175);
    for (uint8_t g = 0; g < 16; g++) {
        CHECK_EQ((int)snap.gate_energies[g], (int)energies[g]);
    }
    std::printf("ok\n");
}

static void test_energy_frame_bad_length() {
    std::printf("test_energy_frame_bad_length ... ");
    ld2420 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);

    // First a valid frame to set known state.
    uint16_t e1[16] = {0};
    e1[0] = 1234;
    s.inject(make_energy_frame(true, 42, e1));
    drain(r, s);
    CHECK(r.dataFrameReceived());
    CHECK_EQ((int)r.targetDistance(), 42);

    // Now a malformed frame: intra_len claims 35 but total bytes only 30
    // (truncated body). The parser should drop it; state should NOT change.
    s.inject({
        0xF4, 0xF3, 0xF2, 0xF1,
        0x23, 0x00,
        0x01, 0xFF, 0xFF,           // presence + distance
        // only 7 bytes of "energies", then a fake footer
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0xF8, 0xF7, 0xF6, 0xF5
    });
    drain(r, s);

    // Liveness timestamp may have updated but the data fields must not.
    CHECK_EQ((int)r.targetDistance(), 42);
    CHECK_EQ((int)r.gateEnergy(0), 1234);
    std::printf("ok\n");
}

static void test_data_frame_resync() {
    std::printf("test_data_frame_resync ... ");
    ld2420 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);

    // Garbage bytes followed by a valid energy frame. The parser scans for
    // the head and should ignore the leading noise.
    s.inject({ 0x12, 0x34, 0xF4, 0x99, 0xF4, 0xF3 });   // bad starts
    uint16_t energies[16] = {0};
    energies[5] = 77;
    auto frame = make_energy_frame(true, 88, energies);
    s.inject(frame);
    drain(r, s);

    CHECK(r.dataFrameReceived());
    CHECK_EQ((int)r.targetDistance(), 88);
    CHECK_EQ((int)r.gateEnergy(5), 77);
    std::printf("ok\n");
}

// ===========================================================================
// Command-side tests
// ===========================================================================

static void test_open_command_mode() {
    std::printf("test_open_command_mode ... ");
    ld2420 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);

    s.inject_response(make_open_cmd_mode_ack(/*proto=*/2, /*bufsize=*/0x0400));
    s.inject_response(make_open_cmd_mode_ack(/*proto=*/2, /*bufsize=*/0x0400));

    CHECK(r.enterCommandMode());
    CHECK_EQ((int)r.protocol_version, 2);
    CHECK_EQ((int)r.tx_rx_buffer_size, 0x0400);
    std::printf("ok\n");
}

static void test_request_firmware_version() {
    std::printf("test_request_firmware_version ... ");
    ld2420 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);

    stage_command_envelope(s, make_firmware_ack("v1.4.14"));
    CHECK(r.requestFirmwareVersion());

    CHECK_EQ((int)r.firmware_version_length, 7);
    CHECK(std::strcmp(r.firmware_version_ascii, "v1.4.14") == 0);
    CHECK_EQ((int)r.firmware_major_version, 1);
    CHECK_EQ((int)r.firmware_minor_version, 4);
    CHECK_EQ((int)r.firmware_patch_version, 14);
    std::printf("ok\n");
}

static void test_write_register() {
    std::printf("test_write_register ... ");
    ld2420 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);
    stage_command_envelope(s, make_short_ack(0x0001));
    CHECK(r.writeRegister(0x0020, 0x0041, 0xC804));
    std::printf("ok\n");
}

static void test_read_register() {
    std::printf("test_read_register ... ");
    ld2420 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);

    stage_command_envelope(s, make_register_read_ack(0xC804));

    uint16_t v = 0;
    CHECK(r.readRegister(0x0020, 0x0041, v));
    CHECK_EQ((int)v, 0xC804);
    std::printf("ok\n");
}

static void test_write_system_parameter() {
    std::printf("test_write_system_parameter ... ");
    ld2420 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);
    stage_command_envelope(s, make_short_ack(0x0012));
    CHECK(r.writeSystemParameter(LD2420_SYS_W_MODE, LD2420_SYS_MODE_MTT));
    std::printf("ok\n");
}

static void test_read_system_parameter() {
    std::printf("test_read_system_parameter ... ");
    ld2420 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);

    stage_command_envelope(s, make_sys_param_read_ack(0xDEADBEEF));

    uint32_t v = 0;
    CHECK(r.readSystemParameter(LD2420_SYS_W_MODE, v));
    CHECK_EQ((unsigned long)v, 0xDEADBEEFUL);
    std::printf("ok\n");
}

static void test_set_get_system_mode() {
    std::printf("test_set_get_system_mode ... ");
    ld2420 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);

    // setSystemMode
    stage_command_envelope(s, make_short_ack(0x0012));
    CHECK(r.setSystemMode(LD2420_SYS_MODE_GR));

    // getSystemMode
    stage_command_envelope(s, make_sys_param_read_ack((uint32_t)LD2420_SYS_MODE_GR));
    uint8_t mode = 0;
    CHECK(r.getSystemMode(mode));
    CHECK_EQ((int)mode, LD2420_SYS_MODE_GR);
    std::printf("ok\n");
}

static void test_write_abd_parameter() {
    std::printf("test_write_abd_parameter ... ");
    ld2420 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);
    stage_command_envelope(s, make_short_ack(0x0007));
    CHECK(r.writeAbdParameter(LD2420_ABD_W_ROI_MIN, 2));
    std::printf("ok\n");
}

static void test_read_abd_parameter() {
    std::printf("test_read_abd_parameter ... ");
    ld2420 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);

    stage_command_envelope(s, make_abd_param_read_ack(0x12340005));
    uint32_t v = 0;
    CHECK(r.readAbdParameter(LD2420_ABD_W_HIGH_THRESHOLD, v));
    CHECK_EQ((unsigned long)v, 0x12340005UL);
    std::printf("ok\n");
}

static void test_set_abd_threshold_packing() {
    std::printf("test_set_abd_threshold_packing ... ");
    ld2420 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);

    // setAbdHighThresholdAtGate(5, 0x1234) should pack to 0x12340005 in
    // the wire payload — the §1.2.4 example. We verify two things:
    //   (a) the public method returns true on success
    //   (b) the readback (via per-gate read word 0x0020+5 = 0x0025) yields
    //       0x1234 (the unpacked threshold) when the radar would have
    //       stored it.
    stage_command_envelope(s, make_short_ack(0x0007));
    CHECK(r.setAbdHighThresholdAtGate(5, 0x1234));

    // Read it back — the per-gate read returns just the threshold (no
    // gate index in the result), so we stash 0x1234 as a 32-bit value.
    stage_command_envelope(s, make_abd_param_read_ack(0x00001234));
    uint16_t v = 0;
    CHECK(r.readAbdHighThresholdAtGate(5, v));
    CHECK_EQ((int)v, 0x1234);
    std::printf("ok\n");
}

static void test_set_abd_roi() {
    std::printf("test_set_abd_roi ... ");
    ld2420 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);
    // setAbdRoi writes two parameter words in ONE 0x0007 command, so still
    // just one ACK to inject.
    stage_command_envelope(s, make_short_ack(0x0007));
    CHECK(r.setAbdRoi(/*min_gate=*/1, /*max_gate=*/12));
    std::printf("ok\n");
}

static void test_request_serial_number() {
    std::printf("test_request_serial_number ... ");
    ld2420 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);

    stage_command_envelope(s, make_sn_ack(0xABCD, 0x12345678));
    CHECK(r.requestSerialNumber());
    CHECK_EQ((int)r.module_identification, 0xABCD);
    CHECK_EQ((unsigned long)r.serial_number, 0x12345678UL);
    std::printf("ok\n");
}

static void test_enter_factory_test_mode() {
    std::printf("test_enter_factory_test_mode ... ");
    ld2420 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);

    // HLK §1.2.9 example: subboard=0, cascaded=2, rx=4, dtype=0 (1DFFT),
    // 1dfft_size=64, chirps=32, downsampling=2.
    stage_command_envelope(s,
        make_factory_test_enter_ack(0, 2, 4, LD2420_FT_DATA_TYPE_1DFFT,
                                    64, 32, 2));
    CHECK(r.enterFactoryTestMode());
    CHECK_EQ((int)r.ft_subboard_model,   0);
    CHECK_EQ((int)r.ft_cascaded_chips,   2);
    CHECK_EQ((int)r.ft_rx_channels,      4);
    CHECK_EQ((int)r.ft_data_type,        LD2420_FT_DATA_TYPE_1DFFT);
    CHECK_EQ((int)r.ft_1dfft_size,       64);
    CHECK_EQ((int)r.ft_chirps_per_frame, 32);
    CHECK_EQ((int)r.ft_downsampling,     2);
    std::printf("ok\n");
}

static void test_exit_factory_test_mode() {
    std::printf("test_exit_factory_test_mode ... ");
    ld2420 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);
    stage_command_envelope(s, make_short_ack(0x0025));
    CHECK(r.exitFactoryTestMode());
    std::printf("ok\n");
}

static void test_send_factory_test_result() {
    std::printf("test_send_factory_test_result ... ");
    ld2420 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);
    stage_command_envelope(s, make_short_ack(0x0026));
    CHECK(r.sendFactoryTestResult(/*address=*/0x0010, /*data=*/0xBEEF));
    std::printf("ok\n");
}

// ===========================================================================
// Edge cases
// ===========================================================================

static void test_ack_status_failure() {
    std::printf("test_ack_status_failure ... ");
    ld2420 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);

    // Open cmd mode succeeds, but the actual READ_VERSION ACK has status=1.
    // requestFirmwareVersion must return false; the firmware_* fields must
    // remain at their default values.
    s.inject_response(make_open_cmd_mode_ack());
    s.inject_response(make_open_cmd_mode_ack());
    s.inject_response(make_short_ack(0x0000, /*status=*/1));
    s.inject_response(make_short_ack(0x00FE));

    CHECK(!r.requestFirmwareVersion());
    CHECK_EQ((int)r.firmware_version_length, 0);
    CHECK(r.firmware_version_ascii[0] == '\0');
    std::printf("ok\n");
}

static void test_ack_wrong_opcode() {
    std::printf("test_ack_wrong_opcode ... ");
    ld2420 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);

    // Open cmd mode succeeds. The READ_VERSION expects ACK 0x0100; we send
    // ACK 0x0124 (factory-test-enter ACK) instead. wait_for_ack_ should
    // time out → method returns false.
    s.inject_response(make_open_cmd_mode_ack());
    s.inject_response(make_open_cmd_mode_ack());
    s.inject_response(make_short_ack(0x0024));   // wrong opcode for expected 0x0000
    s.inject_response(make_short_ack(0x00FE));

    CHECK(!r.requestFirmwareVersion());
    std::printf("ok\n");
}

// ===========================================================================
// main

int main() {
    test_energy_frame_basic();
    test_energy_frame_no_presence();
    test_energy_frame_atomic_snapshot();
    test_energy_frame_bad_length();
    test_data_frame_resync();

    test_open_command_mode();
    test_request_firmware_version();

    test_write_register();
    test_read_register();
    test_write_system_parameter();
    test_read_system_parameter();
    test_set_get_system_mode();
    test_write_abd_parameter();
    test_read_abd_parameter();
    test_set_abd_threshold_packing();
    test_set_abd_roi();
    test_request_serial_number();
    test_enter_factory_test_mode();
    test_exit_factory_test_mode();
    test_send_factory_test_result();

    test_ack_status_failure();
    test_ack_wrong_opcode();

    if (failures == 0) {
        std::printf("\nAll ld2420 parser tests passed.\n");
        return 0;
    }
    std::fprintf(stderr, "\n%d ld2420 parser test failure(s).\n", failures);
    return 1;
}
