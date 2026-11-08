/*
 * Companion Drone Ground Station ESP-NOW Bridge
 *
 * Hardware: ESP32 development board (connected via USB to Ground Laptop)
 * Function: Transparent bidirectional bridge between USB Serial and ESP-NOW (LR mode).
 *
 * Serial Baud: 115200 (configurable)
 * ESP-NOW Mode: WiFi Long Range (LR) 802.11b/g/n LR protocol
 */

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#define SERIAL_BAUD 115200
#define BUFFER_SIZE 240       // ESP-NOW max payload is 250 bytes
#define TX_TIMEOUT_MS 5       // Flush timeout to prevent serial lag
#define LED_PIN 2             // Built-in LED for packet activity

// Target Air ESP32 MAC address (default broadcast, replace with specific MAC for security)
uint8_t airMacAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t serialBuffer[BUFFER_SIZE];
size_t bufferIndex = 0;
unsigned long lastSerialByteTime = 0;

// Callback when data is received over ESP-NOW from Air ESP32
void onDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
    if (len > 0) {
        Serial.write(incomingData, len);
        digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Toggle LED on activity
    }
}

// Callback when data was sent over ESP-NOW
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    // Optional transmission status hook
}

void flushBuffer() {
    if (bufferIndex > 0) {
        esp_now_send(airMacAddress, serialBuffer, bufferIndex);
        bufferIndex = 0;
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
}

void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    Serial.begin(SERIAL_BAUD);
    while (!Serial && millis() < 3000);

    // Set WiFi to Station mode
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // Enable Long Range (LR) mode for maximum outdoor telemetry range
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR | WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G);

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("[ESP-NOW] Init Failed!");
        return;
    }

    esp_now_register_recv_cb(onDataRecv);
    esp_now_register_send_cb(onDataSent);

    // Register Air peer
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, airMacAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("[ESP-NOW] Failed to add peer");
    }

    digitalWrite(LED_PIN, HIGH);
}

void loop() {
    // Read from USB Serial and buffer for transmission
    while (Serial.available()) {
        uint8_t b = Serial.read();
        serialBuffer[bufferIndex++] = b;
        lastSerialByteTime = millis();

        if (bufferIndex >= BUFFER_SIZE) {
            flushBuffer();
        }
    }

    // Flush remaining buffer if timeout elapsed
    if (bufferIndex > 0 && (millis() - lastSerialByteTime >= TX_TIMEOUT_MS)) {
        flushBuffer();
    }
}

