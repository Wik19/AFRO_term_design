#include <WiFi.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include "driver/i2s.h"

// --- Wi-Fi Configuration ---
// IMPORTANT: Replace with your phone hotspot's SSID and Password
const char *ssid = "incandescent_spot";
const char *password = "987654321";
const uint16_t serverPort = 8080; // Port for the TCP server

// --- Sensor Pin Definitions ---
// IMU (LSM6DSO32) SPI Pins
#define IMU_CS_PIN 4
#define IMU_SCK_PIN 6
#define IMU_MISO_PIN 2
#define IMU_MOSI_PIN 5

// Microphone (INMP441) I2S Pins
#define I2S_MIC_SERIAL_CLOCK_PIN 10 // SCK
#define I2S_MIC_WORD_SELECT_PIN 9   // WS / LRCL
#define I2S_MIC_SERIAL_DATA_PIN 8   // SD / DOUT

// --- Sensor Configuration ---
// Use the highest rate defined in the Adafruit library (check library header if needed)
// LSM6DS_RATE_1666_HZ is typically the highest standard one defined.
#define IMU_ODR LSM6DS_RATE_1_66K_HZ // Target ODR: 1.66 kHz
#define I2S_SAMPLE_RATE 16000
#define I2S_BITS_PER_SAMPLE I2S_BITS_PER_SAMPLE_32BIT
#define I2S_PORT_NUM I2S_NUM_0
#define MIC_AUDIO_CHUNK_SAMPLES 256 // Number of samples per audio packet

// --- Global Objects ---
Adafruit_LSM6DSOX sox;
WiFiServer tcpServer(serverPort);
WiFiClient client; // Holds the current connected client
int32_t i2s_read_buffer[MIC_AUDIO_CHUNK_SAMPLES]; // Buffer for I2S reads

// --- Binary Packet Structure Definition ---
const uint8_t PACKET_TYPE_IMU = 0x01;
const uint8_t PACKET_TYPE_AUDIO = 0x02;

// Structure for IMU data packet (ensure packing)
#pragma pack(push, 1) // Ensure compiler doesn't add padding
struct ImuPacket {
    uint8_t type = PACKET_TYPE_IMU;
    uint32_t timestamp_ms;
    float accX;
    float accY;
    float accZ;
    float gyroX;
    float gyroY;
    float gyroZ;
};
#pragma pack(pop)

// Structure for Audio data packet (header + flexible data)
#pragma pack(push, 1)
struct AudioPacketHeader {
    uint8_t type = PACKET_TYPE_AUDIO;
    uint32_t timestamp_ms;
    uint16_t num_samples = MIC_AUDIO_CHUNK_SAMPLES;
};
#pragma pack(pop)
// Audio samples (int32_t) will be sent immediately after the header

// --- Function Prototypes ---
void setup_wifi();
void setup_imu();
void setup_microphone();
void sendImuData(const ImuPacket& packet);
void sendAudioData(const AudioPacketHeader& header, const int32_t* samples);

// --- Setup Function ---
void setup() {
    Serial.begin(115200); // Keep Serial for debugging/status messages
    while (!Serial) {
        delay(10);
    }
    Serial.println("\nESP32-C3 Wi-Fi TCP Sensor Server");

    setup_wifi();
    setup_imu();
    setup_microphone();

    Serial.println("Setup finished. Starting TCP server...");
    tcpServer.begin();
    Serial.printf("TCP server started on port %d\n", serverPort);
}

// --- Main Loop ---
void loop() {
    // Check if a new client has connected
    if (!client || !client.connected()) {
        // If client was connected, print disconnect message
        if(client && !client.connected()){
            Serial.println("Client disconnected.");
            client.stop(); // Ensure resources are freed
        }
        client = tcpServer.available(); // Check for new client connection
        if (client) {
            Serial.printf("New client connected: %s\n", client.remoteIP().toString().c_str());
        } else {
            // No client connected, maybe add a small delay or check less frequently
            delay(10); // Prevent tight loop when no client
            return;   // Go back to start of loop
        }
    }

    // --- If we have a connected client ---

    // 1. Read and Send IMU Data
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp; // Temperature is read by getEvent but not sent

    // Use getEvent() which checks internal flags for new data availability
    if (sox.getEvent(&accel, &gyro, &temp)) {
        ImuPacket imuPacket;
        imuPacket.timestamp_ms = millis();
        imuPacket.accX = accel.acceleration.x;
        imuPacket.accY = accel.acceleration.y;
        imuPacket.accZ = accel.acceleration.z;
        imuPacket.gyroX = gyro.gyro.x;
        imuPacket.gyroY = gyro.gyro.y;
        imuPacket.gyroZ = gyro.gyro.z;
        sendImuData(imuPacket);
    }

    // 2. Read and Send Audio Data
    size_t bytes_read = 0;
    esp_err_t i2s_err = i2s_read(I2S_PORT_NUM,
                                 (void *)i2s_read_buffer,
                                 MIC_AUDIO_CHUNK_SAMPLES * sizeof(int32_t),
                                 &bytes_read,
                                 pdMS_TO_TICKS(10)); // Short timeout, non-blocking preferred

    if (i2s_err == ESP_OK && bytes_read == MIC_AUDIO_CHUNK_SAMPLES * sizeof(int32_t)) {
        AudioPacketHeader audioHeader;
        audioHeader.timestamp_ms = millis(); // Timestamp for the start of this chunk
        // audioHeader.num_samples is already set
        sendAudioData(audioHeader, i2s_read_buffer);
    } else if (i2s_err != ESP_OK && i2s_err != ESP_ERR_TIMEOUT) {
         // Don't log timeout errors as they are expected if buffer isn't full
         Serial.printf("I2S read error: %d\n", i2s_err);
    }

    // Small delay to prevent overwhelming the loop/network stack if data reads are very fast
    // Especially important if the client disconnects/reconnects frequently or if write buffer fills
    // delayMicroseconds(100); // e.g., 0.1 ms delay - adjust as needed
}

// --- Helper Functions ---

void setup_wifi() {
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.mode(WIFI_STA); // Set WiFi to station mode
    WiFi.begin(ssid, password);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) { // Timeout after ~10 seconds
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected!");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\nFailed to connect to WiFi. Please check credentials or hotspot.");
        // Handle connection failure (e.g., restart, enter config mode)
        // For now, just halt
         while(1) delay(1000);
    }
}

void setup_imu() {
    Serial.println("Setting up IMU (LSM6DSO32)...");
    SPI.begin(IMU_SCK_PIN, IMU_MISO_PIN, IMU_MOSI_PIN, -1);

    if (!sox.begin_SPI(IMU_CS_PIN, &SPI)) {
        Serial.println("Failed to find LSM6DSOX chip via SPI. Check wiring.");
        while (1) delay(10);
    }
    Serial.println("LSM6DSOX Found!");

    // Set Accelerometer Range (e.g., 16G for vibration)
    sox.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
    Serial.print("Accelerometer range set to: +-16G\n");

    // Set Gyroscope Range (e.g., 2000 DPS)
    sox.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
    Serial.print("Gyro range set to: 2000 DPS\n");

    // Set Output Data Rate (ODR) - Corrected to library-defined constant
    sox.setAccelDataRate(IMU_ODR);
    sox.setGyroDataRate(IMU_ODR);
    // Update the print statement to reflect the actual rate
    Serial.printf("IMU ODR set to: %d Hz (approx)\n", 1666); // Reflecting LSM6DS_RATE_1666_HZ

    Serial.println("IMU Setup Complete.");
    delay(100); // Allow sensor to stabilize
}

void setup_microphone() {
    Serial.println("Setting up Microphone (INMP441)...");
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = I2S_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, // Assuming L/R pin is LOW
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,  // More buffers might help with high data rates
        .dma_buf_len = MIC_AUDIO_CHUNK_SAMPLES, // DMA buffer length matches our chunk size
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
        };

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_MIC_SERIAL_CLOCK_PIN,
        .ws_io_num = I2S_MIC_WORD_SELECT_PIN,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_MIC_SERIAL_DATA_PIN
    };

    esp_err_t err = i2s_driver_install(I2S_PORT_NUM, &i2s_config, 0, NULL);
    if (err != ESP_OK) {
        Serial.printf("Failed to install I2S driver: %d\n", err);
        while (1) delay(10);
    }

    err = i2s_set_pin(I2S_PORT_NUM, &pin_config);
    if (err != ESP_OK) {
        Serial.printf("Failed to set I2S pins: %d\n", err);
        while (1) delay(10);
    }

    Serial.println("Waiting for microphone to settle...");
    delay(200); // Allow INMP441 power-up time
    // Clear the I2S buffer potentially filled during setup
    i2s_zero_dma_buffer(I2S_PORT_NUM);
    Serial.println("Microphone Setup Complete.");
}

// Function to send IMU data packet over TCP
void sendImuData(const ImuPacket& packet) {
    if (client && client.connected()) {
        size_t bytesSent = client.write((const uint8_t*)&packet, sizeof(packet));
        if (bytesSent != sizeof(packet)) {
            Serial.println("Error sending IMU packet or client disconnected.");
            client.stop(); // Close connection on error
        }
    }
}

// Function to send Audio data packet over TCP
void sendAudioData(const AudioPacketHeader& header, const int32_t* samples) {
     if (client && client.connected()) {
        // Send header first
        size_t headerBytesSent = client.write((const uint8_t*)&header, sizeof(header));
        if (headerBytesSent != sizeof(header)) {
             Serial.println("Error sending Audio header or client disconnected.");
             client.stop();
             return; // Don't attempt to send samples if header failed
        }

        // Send audio samples immediately after
        size_t samplesBytesSent = client.write((const uint8_t*)samples, header.num_samples * sizeof(int32_t));
         if (samplesBytesSent != header.num_samples * sizeof(int32_t)) {
             Serial.println("Error sending Audio samples or client disconnected.");
             client.stop();
         }
    }
}
