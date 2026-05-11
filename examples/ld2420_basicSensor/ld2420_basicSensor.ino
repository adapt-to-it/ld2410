/*
 *	Smoke-test sketch for the HLK-LD2420 driver.
 *
 *	What it does:
 *	  1. Opens the radar's UART (115200 8N1 default — see HLK product manual).
 *	  2. Calls begin() which probes the firmware version by entering
 *	     command mode, reading 0x0000, and leaving command mode.
 *	  3. Prints the firmware version string and protocol-version / buffer
 *	     fields populated by enterCommandMode().
 *	  4. Loops calling read() and, once an energy frame has been parsed,
 *	     prints presence + distance + gate-4 FFT energy. The radar must be
 *	     in a system mode that emits per-gate energy frames; see
 *	     setSystemMode() in docs/10-api-ld2420.md.
 *
 *	STATUS: UNVERIFIED ON HARDWARE — see banner in src/ld2420.h.
 */
#include <Arduino.h>
#include <ld2420.h>

// Adjust to your wiring. Default in this sketch matches the LD2410 examples
// for ESP32: UART2 on GPIO16 (RX) / GPIO17 (TX).
#if defined(ESP32)
HardwareSerial & radarSerial = Serial2;
constexpr int8_t RADAR_RX_PIN = 16;
constexpr int8_t RADAR_TX_PIN = 17;
#elif defined(ARDUINO_ARCH_RP2040)
auto & radarSerial = Serial1;
#else
auto & radarSerial = Serial1;
#endif

ld2420 radar;

void setup() {
	Serial.begin(115200);
	while (!Serial && millis() < 3000) { /* wait briefly for USB CDC */ }
	Serial.println(F("LD2420 smoke test"));

#if defined(ESP32)
	radarSerial.begin(115200, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
#else
	radarSerial.begin(115200);
#endif

	radar.debug(Serial);
	if (radar.begin(radarSerial)) {
		Serial.print(F("firmware: "));
		Serial.println(radar.firmware_version_ascii);
		Serial.print(F("protocol version: "));
		Serial.println(radar.protocol_version);
		Serial.print(F("buffer size: "));
		Serial.println(radar.tx_rx_buffer_size);
	} else {
		Serial.println(F("LD2420 not responding"));
	}
}

void loop() {
	radar.read();

	// Throttle status print to ~1 Hz so the monitor stays readable; the
	// radar itself emits energy frames at its own (configurable) rate.
	static uint32_t last_print = 0;
	if (millis() - last_print > 1000) {
		last_print = millis();
		if (radar.dataFrameReceived()) {
			LD2420TargetState state;
			radar.snapshotTargetState(state);
			Serial.print(F("presence="));
			Serial.print(state.presence ? F("yes") : F("no "));
			Serial.print(F("  distance="));
			Serial.print(state.distance_cm);
			Serial.print(F(" cm  g4_energy="));
			Serial.println(state.gate_energies[4]);
		}
	}
}
