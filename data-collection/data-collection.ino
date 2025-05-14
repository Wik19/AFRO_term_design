#include <WiFi.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include "driver/i2s.h"
#include <vector>
#include <cstring> // For memcpy

// --- Wi-Fi Configuration ---
const char *ssid = "incandescent_spot";
const char *password = "987654321";
const uint16_t serverPort = 8080;

// --- Sensor Pin Definitions ---
#define IMU_CS_PIN 4
#define IMU_SCK_PIN 6
#define IMU_MISO_PIN 2
#define IMU_MOSI_PIN 5

#define I2S_MIC_SERIAL_CLOCK_PIN 10
#define I2S_MIC_WORD_SELECT_PIN 9
#define I2S_MIC_SERIAL_DATA_PIN 8

// --- Sensor Configuration ---
#define IMU_ODR LSM6DS_RATE_416_HZ
#define I2S_SAMPLE_RATE 16000
#define I2S_BITS_PER_SAMPLE I2S_BITS_PER_SAMPLE_32BIT
#define I2S_PORT_NUM I2S_NUM_0
#define MIC_AUDIO_CHUNK_SAMPLES 256 // Number of int32_t samples per audio packet

// --- Application State & Buffering Configuration ---
enum AppState {
    WAITING_FOR_CLIENT,
    RECORDING,
    SENDING_DATA
};
AppState currentState = WAITING_FOR_CLIENT;

const size_t TARGET_AUDIO_PACKETS = 100; // Number of combined audio data packets to collect

// --- Global Objects ---
Adafruit_LSM6DSOX sox;
WiFiServer tcpServer(serverPort);
WiFiClient client;

// --- Binary Packet Structure Definition ---
const uint8_t PACKET_TYPE_IMU = 0x01;
const uint8_t PACKET_TYPE_AUDIO = 0x02;

#pragma pack(push, 1)
struct ImuPacket {
    uint8_t type = PACKET_TYPE_IMU;
    uint32_t timestamp_ms;
    float accX, accY, accZ;
    float gyroX, gyroY, gyroZ;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct AudioDataPacket { // <<< NEW COMBINED AUDIO PACKET
    uint8_t type = PACKET_TYPE_AUDIO;
    uint32_t timestamp_ms;
    uint16_t num_samples = MIC_AUDIO_CHUNK_SAMPLES; // Should always be this value
    int32_t samples[MIC_AUDIO_CHUNK_SAMPLES];     // Actual sample data embedded
};
#pragma pack(pop)

// --- Data Buffers ---
std::vector<ImuPacket> imuBuffer;
std::vector<AudioDataPacket> audioDataBuffer; // <<< SINGLE BUFFER FOR AUDIO

// Buffer Capacities
// IMU: 416 Hz * 1.6 s (approx for 100 audio packets) = 665.6 packets. Add margin.
const size_t IMU_BUFFER_EXPECTED_CAPACITY = (size_t)(416 * 1.6 * 1.15);         // ~765
const size_t AUDIO_PACKETS_EXPECTED_CAPACITY = TARGET_AUDIO_PACKETS;            // 100

// Temporary buffer for I2S reads, remains the same
int32_t i2s_read_chunk_buffer[MIC_AUDIO_CHUNK_SAMPLES];

// --- Function Prototypes ---
void setup_wifi();
void setup_imu();
void setup_microphone();
void sendBufferedData();
void clearBuffers();

// --- Setup Function ---
void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    Serial.println("\nESP32-C3 Sensor Server - Combined Audio Packets");

    Serial.printf("Initial free heap: %u bytes\n", ESP.getFreeHeap());
    Serial.printf("Target audio packets: %zu\n", TARGET_AUDIO_PACKETS);
    Serial.printf("Size of ImuPacket: %zu bytes\n", sizeof(ImuPacket));
    Serial.printf("Size of AudioDataPacket: %zu bytes\n", sizeof(AudioDataPacket)); // Will be ~1031 bytes
    
    // Calculate total expected memory for buffers
    size_t imu_mem = IMU_BUFFER_EXPECTED_CAPACITY * sizeof(ImuPacket);
    size_t audio_mem = AUDIO_PACKETS_EXPECTED_CAPACITY * sizeof(AudioDataPacket);
    Serial.printf("Expected capacities: IMU=%zu (%.1f KB), AudioPackets=%zu (%.1f KB)\n",
                  IMU_BUFFER_EXPECTED_CAPACITY, imu_mem / 1024.0,
                  AUDIO_PACKETS_EXPECTED_CAPACITY, audio_mem / 1024.0);
    Serial.printf("Total expected buffer memory: %.1f KB\n", (imu_mem + audio_mem) / 1024.0);


    imuBuffer.reserve(IMU_BUFFER_EXPECTED_CAPACITY);
    audioDataBuffer.reserve(AUDIO_PACKETS_EXPECTED_CAPACITY);
    
    Serial.printf("Actual reserved capacity: IMU=%zu, AudioDataPackets=%zu\n",
                  imuBuffer.capacity(), audioDataBuffer.capacity());
    Serial.printf("Free heap after reserve: %u bytes\n", ESP.getFreeHeap());

    if (imuBuffer.capacity() < IMU_BUFFER_EXPECTED_CAPACITY ||
        audioDataBuffer.capacity() < AUDIO_PACKETS_EXPECTED_CAPACITY) {
        Serial.println("!!! WARNING: Buffer reservation failed or partial. May run out of memory. !!!");
    }

    setup_wifi();
    setup_imu();
    setup_microphone();

    Serial.println("Setup finished. Starting TCP server...");
    tcpServer.begin();
    Serial.printf("TCP server started on port %d\n", serverPort);
    currentState = WAITING_FOR_CLIENT;
    Serial.printf("Free heap at start of loop: %u bytes\n", ESP.getFreeHeap());
}

// --- Main Loop ---
void loop() {
    if (!client || !client.connected()) {
        if (client && currentState != WAITING_FOR_CLIENT) { Serial.println("Client disconnected."); client.stop(); }
        currentState = WAITING_FOR_CLIENT;
        client = tcpServer.available();
        if (client) {
            Serial.printf("New client: %s. Heap: %u\n", client.remoteIP().toString().c_str(), ESP.getFreeHeap());
            Serial.println("To RECORDING.");
            clearBuffers();
            currentState = RECORDING;
        } else {
            delay(10); return;
        }
    }

    if (!client || !client.connected()) { currentState = WAITING_FOR_CLIENT; return; }

    switch (currentState) {
        case RECORDING: {
            sensors_event_t accel, gyro, temp;
            if (sox.getEvent(&accel, &gyro, &temp)) {
                if (imuBuffer.size() < imuBuffer.capacity()) {
                    ImuPacket pkt; pkt.timestamp_ms = millis();
                    pkt.accX = accel.acceleration.x; pkt.accY = accel.acceleration.y; pkt.accZ = accel.acceleration.z;
                    pkt.gyroX = gyro.gyro.x; pkt.gyroY = gyro.gyro.y; pkt.gyroZ = gyro.gyro.z;
                    imuBuffer.push_back(pkt);
                } else { Serial.println("IMU buffer capacity!"); }
            }

            if (audioDataBuffer.size() < TARGET_AUDIO_PACKETS) {
                size_t bytes_read = 0;
                esp_err_t i2s_err = i2s_read(I2S_PORT_NUM, (void *)i2s_read_chunk_buffer,
                                             MIC_AUDIO_CHUNK_SAMPLES * sizeof(int32_t),
                                             &bytes_read, pdMS_TO_TICKS(5));

                if (i2s_err == ESP_OK && bytes_read == MIC_AUDIO_CHUNK_SAMPLES * sizeof(int32_t)) {
                    if (audioDataBuffer.size() < audioDataBuffer.capacity()) {
                        AudioDataPacket audioPkt;
                        audioPkt.timestamp_ms = millis();
                        // audioPkt.num_samples is already MIC_AUDIO_CHUNK_SAMPLES by default
                        memcpy(audioPkt.samples, i2s_read_chunk_buffer, MIC_AUDIO_CHUNK_SAMPLES * sizeof(int32_t));
                        audioDataBuffer.push_back(audioPkt);
                    } else { Serial.println("AudioDataPacket buffer capacity!"); }
                } else if (i2s_err != ESP_OK && i2s_err != ESP_ERR_TIMEOUT) {
                    Serial.printf("I2S read error: %d\n", i2s_err);
                }
            }

            if (audioDataBuffer.size() >= TARGET_AUDIO_PACKETS) {
                Serial.printf("Target audio packets (%zu) reached. IMU: %zu\n",
                              TARGET_AUDIO_PACKETS, imuBuffer.size());
                Serial.println("To SENDING_DATA.");
                currentState = SENDING_DATA;
            }
            break;
        }

        case SENDING_DATA: {
            sendBufferedData();
            if (!client || !client.connected()) { Serial.println("Client lost during send. To WAITING."); currentState = WAITING_FOR_CLIENT; }
            else {
                Serial.println("Batch sent. To RECORDING for next batch.");
                clearBuffers();
                currentState = RECORDING;
            }
            break;
        }
        case WAITING_FOR_CLIENT: default: delay(10); break;
    }
}

void clearBuffers() {
    imuBuffer.clear();
    audioDataBuffer.clear();
}

void sendBufferedData() {
    if (!client || !client.connected()) {
        Serial.println("Send: Client not connected."); currentState = WAITING_FOR_CLIENT; clearBuffers(); return;
    }

    Serial.printf("Sending %zu IMU packets...\n", imuBuffer.size());
    for (const auto& packet : imuBuffer) {
        if (!client.connected()) { Serial.println("Client lost mid-IMU."); clearBuffers(); currentState = WAITING_FOR_CLIENT; return; }
        if (client.write((const uint8_t*)&packet, sizeof(packet)) != sizeof(packet)) {
            Serial.println("IMU send error."); client.stop(); clearBuffers(); currentState = WAITING_FOR_CLIENT; return;
        }
    }

    Serial.printf("Sending %zu AudioDataPackets...\n", audioDataBuffer.size());
    for (const auto& audioPkt : audioDataBuffer) {
        if (!client.connected()) { Serial.println("Client lost mid-AudioData."); clearBuffers(); currentState = WAITING_FOR_CLIENT; return; }
        if (client.write((const uint8_t*)&audioPkt, sizeof(audioPkt)) != sizeof(audioPkt)) {
            Serial.println("AudioDataPacket send error."); client.stop(); clearBuffers(); currentState = WAITING_FOR_CLIENT; return;
        }
    }
    Serial.println("Batch data sent successfully.");
}

// --- Helper Functions (setup_wifi, setup_imu, setup_microphone) ---
// These remain largely the same as in the previous version.
// Ensure IMU_ODR is correctly set in setup_imu.
void setup_wifi() { /* ... Same as before ... */ 
    Serial.print("Connecting to WiFi: "); Serial.println(ssid);
    WiFi.mode(WIFI_STA); WiFi.begin(ssid, password);
    Serial.print("Connecting");
    for (int attempts = 0; WiFi.status() != WL_CONNECTED && attempts < 30; attempts++) {
        delay(500); Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected!");
        Serial.print("IP address: "); Serial.println(WiFi.localIP());
    } else {
        Serial.println("\nFailed to connect to WiFi. Halting."); while(1) delay(1000);
    }
}

void setup_imu() { /* ... Same as before, ensuring IMU_ODR = LSM6DS_RATE_416_HZ ... */
    Serial.println("Setting up IMU (LSM6DSOX)...");
    SPI.begin(IMU_SCK_PIN, IMU_MISO_PIN, IMU_MOSI_PIN, -1);
    if (!sox.begin_SPI(IMU_CS_PIN, &SPI)) {
        Serial.println("Failed to find LSM6DSOX. Halting."); while (1) delay(10);
    }
    Serial.println("LSM6DSOX Found!");
    sox.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
    sox.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
    sox.setAccelDataRate(IMU_ODR); 
    sox.setGyroDataRate(IMU_ODR);  
    
    float odr_hz_val;
    switch(IMU_ODR) {
        case LSM6DS_RATE_SHUTDOWN: odr_hz_val = 0; break; case LSM6DS_RATE_12_5_HZ: odr_hz_val = 12.5; break;
        case LSM6DS_RATE_26_HZ: odr_hz_val = 26; break; case LSM6DS_RATE_52_HZ: odr_hz_val = 52; break;
        case LSM6DS_RATE_104_HZ: odr_hz_val = 104; break; case LSM6DS_RATE_208_HZ: odr_hz_val = 208; break;
        case LSM6DS_RATE_416_HZ: odr_hz_val = 416; break; 
        case LSM6DS_RATE_833_HZ: odr_hz_val = 833; break; case LSM6DS_RATE_1_66K_HZ: odr_hz_val = 1660; break;
        case LSM6DS_RATE_3_33K_HZ: odr_hz_val = 3330; break; case LSM6DS_RATE_6_66K_HZ: odr_hz_val = 6660; break;
        default: odr_hz_val = -1;
    }
    Serial.printf("IMU Config: Accel Range 16G, Gyro Range 2000DPS, ODR ~%.0f Hz\n", odr_hz_val);
    delay(100);
}

void setup_microphone() { /* ... Same as before ... */
    Serial.println("Setting up Microphone (INMP441)...");
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX), .sample_rate = I2S_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE, .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S, .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8, .dma_buf_len = MIC_AUDIO_CHUNK_SAMPLES, 
        .use_apll = false, .tx_desc_auto_clear = false, .fixed_mclk = 0
    };
    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_MIC_SERIAL_CLOCK_PIN, .ws_io_num = I2S_MIC_WORD_SELECT_PIN,
        .data_out_num = I2S_PIN_NO_CHANGE, .data_in_num = I2S_MIC_SERIAL_DATA_PIN
    };
    if (i2s_driver_install(I2S_PORT_NUM, &i2s_config, 0, NULL) != ESP_OK) {
        Serial.println("Failed to install I2S driver. Halting."); while (1) delay(10);
    }
    if (i2s_set_pin(I2S_PORT_NUM, &pin_config) != ESP_OK) {
        Serial.println("Failed to set I2S pins. Halting."); while (1) delay(10);
    }
    delay(200);
    i2s_zero_dma_buffer(I2S_PORT_NUM);
    Serial.println("Microphone Setup Complete.");
}