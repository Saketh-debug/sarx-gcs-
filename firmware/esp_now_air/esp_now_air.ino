/*
 * Companion Drone Air ESP-NOW Bridge
 *
 * Hardware: ESP32 development board (mounted on drone, wired to Raspberry Pi 5 UART)
 * Function: Transparent bidirectional bridge between Pi UART and ESP-NOW (LR mode).
 *
 * Pi UART Connection:
 *   ESP32 TX (GPIO 17) -> Pi 5 RX (GPIO 15 / UART0 RX / Pin 10)
 *   ESP32 RX (GPIO 16) -> Pi 5 TX (GPIO 14 / UART0 TX / Pin 8)
 *   GND                -> Pi 5 GND
 *
 * ESP-NOW Mode: WiFi Long Range (LR) 802.11b/g/n LR protocol
 */

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#define UART_BAUD 115200
#define BUFFER_SIZE 240       // ESP-NOW max payload is 250 bytes
#define TX_TIMEOUT_MS 5       // Flush timeout to prevent serial lag
#define LED_PIN 2             // Built-in LED for packet activity

#define PI_UART_RX_PIN 16     // ESP32 RX <- Pi TX
#define PI_UART_TX_PIN 17     // ESP32 TX -> Pi RX

HardwareSerial PiSerial(2);

// Target Ground ESP32 MAC address (default broadcast, replace with specific MAC for security)
uint8_t groundMacAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t serialBuffer[BUFFER_SIZE];
size_t bufferIndex = 0;
unsigned long lastSerialByteTime = 0;

// Callback when data is received over ESP-NOW from Ground ESP32
void onDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
    if (len > 0) {
        PiSerial.write(incomingData, len);
        digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Toggle LED on activity
    }
}

// Callback when data was sent over ESP-NOW
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    // Optional transmission status hook
}

void flushBuffer() {
    if (bufferIndex > 0) {
        esp_now_send(groundMacAddress, serialBuffer, bufferIndex);
        bufferIndex = 0;
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
}

void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Initialize Pi UART on HardwareSerial 2
    PiSerial.begin(UART_BAUD, SERIAL_8N1, PI_UART_RX_PIN, PI_UART_TX_PIN);

    // Set WiFi to Station mode
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // Enable Long Range (LR) mode for maximum outdoor telemetry range
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR | WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G);

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        return;
    }

    esp_now_register_recv_cb(onDataRecv);
    esp_now_register_send_cb(onDataSent);

    // Register Ground peer
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, groundMacAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    esp_now_add_peer(&peerInfo);

    digitalWrite(LED_PIN, HIGH);
}

void loop() {
    // Read from Pi UART and buffer for transmission over ESP-NOW
    while (PiSerial.available()) {
        uint8_t b = PiSerial.read();
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

