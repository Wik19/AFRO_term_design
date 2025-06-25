#include <WiFi.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include "driver/i2s.h"
#include <vector>
#include <cstring>

// --- Wi-Fi Configuration ---
const char *ssid = "LAPTOP-49D1CUOP 8227";
const char *password = "4<q984E8";
const uint16_t serverPort = 8080;

// --- Sensor Pin Definitions ---
#define IMU_CS_PIN 10 //git
#define IMU_SCK_PIN 6 //git
#define IMU_MISO_PIN 2 //git
#define IMU_MOSI_PIN 7 //git

#define I2S_MIC_SERIAL_CLOCK_PIN 0 // git
#define I2S_MIC_WORD_SELECT_PIN 1 //git
#define I2S_MIC_SERIAL_DATA_PIN 3 //git

// --- Interrupt Pin Definition ---
#define IMU_INT_PIN 5 //git 
volatile bool imuDataReady = false;

// --- Sensor Configuration ---
#define IMU_ODR LSM6DS_RATE_416_HZ
#define I2S_SAMPLE_RATE 16000
#define I2S_BITS_PER_SAMPLE I2S_BITS_PER_SAMPLE_32BIT
#define I2S_PORT_NUM I2S_NUM_0
#define MIC_AUDIO_CHUNK_SAMPLES 256

// --- Application State & Buffering Configuration ---
enum AppState {
    AWAITING_CONNECTION,
    AWAITING_COMMAND,
    STREAMING_IMU,
    STREAMING_AUDIO
};
AppState currentState = AWAITING_CONNECTION;

const size_t TARGET_IMU_PACKETS = 3*416;

// <<< NEW >>> Define the size of our audio batches
const size_t AUDIO_BATCH_PACKET_COUNT = 50;

// --- Global Objects ---
Adafruit_LSM6DSOX sox;
WiFiServer tcpServer(serverPort);
WiFiClient client;

uint32_t imuPacketCounter = 0;

// --- Binary Packet Structure Definition ---
const uint8_t PACKET_TYPE_IMU = 0x01;
const uint8_t PACKET_TYPE_AUDIO = 0x02;
const uint8_t CMD_REQUEST_IMU = 0x11;
const uint8_t CMD_REQUEST_AUDIO = 0x22;

#pragma pack(push, 1)
struct ImuPacket {
    uint8_t type = PACKET_TYPE_IMU;
    uint32_t packetIndex;
    float accX, accY, accZ;
    float gyroX, gyroY, gyroZ;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct AudioDataPacket {
    uint8_t type = PACKET_TYPE_AUDIO;
    uint32_t timestamp_ms;
    uint16_t num_samples = MIC_AUDIO_CHUNK_SAMPLES;
    int32_t samples[MIC_AUDIO_CHUNK_SAMPLES];
};
#pragma pack(pop)

// --- Data Buffers ---
std::vector<ImuPacket> imuBuffer;
const size_t IMU_BUFFER_CAPACITY = TARGET_IMU_PACKETS;

// <<< NEW >>> Statically allocate our two large audio buffers for the ping-pong mechanism
static AudioDataPacket audioBufferA[AUDIO_BATCH_PACKET_COUNT];
static AudioDataPacket audioBufferB[AUDIO_BATCH_PACKET_COUNT];

// <<< NEW >>> FreeRTOS objects for concurrent streaming
QueueHandle_t audioBufferQueue; // This queue will hold POINTERS to the full buffers
TaskHandle_t audioCollectorTaskHandle;
TaskHandle_t wifiSenderTaskHandle;
volatile bool isStreamingAudio = false; // Flag to control the tasks


// --- Function Prototypes ---
void setup_wifi();
void setup_imu();
void setup_microphone();
void sendImuData();
void clearBuffers();

// --- Interrupt Service Routine (ISR) ---
void IRAM_ATTR imuInterruptHandler() {
  imuDataReady = true;
}

void audioCollectorTask(void *pvParameters) {
    // Start by using Buffer A
    AudioDataPacket* currentBuffer = audioBufferA;

    for (;;) { // Loop forever
        if (isStreamingAudio) {
            // Fill the current buffer with 50 packets
            for (int i = 0; i < AUDIO_BATCH_PACKET_COUNT; i++) {
                size_t bytes_read = 0;
                // Block and wait for a chunk of audio from the I2S driver
                i2s_read(I2S_PORT_NUM, (void*)currentBuffer[i].samples, 
                         MIC_AUDIO_CHUNK_SAMPLES * sizeof(int32_t), 
                         &bytes_read, portMAX_DELAY);
                
                // Add a timestamp to this individual packet
                currentBuffer[i].timestamp_ms = millis();
            }

            // Now that the buffer is full, send a POINTER to it to the sender task
            xQueueSend(audioBufferQueue, &currentBuffer, portMAX_DELAY);

            // Immediately switch to the other buffer to start collecting new data (the "pong")
            // This ensures no gap in collection.
            if (currentBuffer == audioBufferA) {
                currentBuffer = audioBufferB;
            } else {
                currentBuffer = audioBufferA;
            }

        } else {
            vTaskDelay(pdMS_TO_TICKS(100)); // Wait if not streaming
        }
    }
}

// <<< NEW >>> Task 2: The Wi-Fi Sender (Consumer)
// This task's only job is to wait for a full buffer and send it.
void wifiSenderTask(void *pvParameters) {
    AudioDataPacket* bufferToSend; // A pointer to hold the buffer we receive from the queue

    for (;;) { // Loop forever
        if (isStreamingAudio && client && client.connected()) {
            // Wait until a pointer to a full buffer appears on the queue
            if (xQueueReceive(audioBufferQueue, &bufferToSend, portMAX_DELAY) == pdPASS) {
                // We received a full buffer! Now, send all 50 packets.
                for (int i = 0; i < AUDIO_BATCH_PACKET_COUNT; i++) {
                    client.write((const uint8_t*)&bufferToSend[i], sizeof(AudioDataPacket));
                }
                Serial.printf("Sent audio batch of %d packets.\n", AUDIO_BATCH_PACKET_COUNT);
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(100)); // Wait if not streaming or not connected
        }
    }
}


// --- Setup Function ---
void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    Serial.println("\nESP32-C3 Sensor Server - Dual Task Ping-Pong");

    imuBuffer.reserve(IMU_BUFFER_CAPACITY);

    pinMode(IMU_INT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(IMU_INT_PIN), imuInterruptHandler, RISING);

    setup_wifi();
    setup_imu();
    setup_microphone();

    // Queue can hold up to 2 pointers (one for each buffer)
    audioBufferQueue = xQueueCreate(2, sizeof(AudioDataPacket*));

    // Create tasks pinned to Core 0. Let Wi-Fi and other system stuff have Core 1 if needed.
    // Collector task has higher priority (2) than Sender task (1)
    xTaskCreatePinnedToCore(audioCollectorTask, "AudioCollector", 4096, NULL, 2, &audioCollectorTaskHandle, 0);
    xTaskCreatePinnedToCore(wifiSenderTask, "WiFiSender", 4096, NULL, 1, &wifiSenderTaskHandle, 0);

    Serial.println("Setup finished. Starting TCP server...");
    tcpServer.begin();
    Serial.printf("TCP server started on port %d\n", serverPort);
    currentState = AWAITING_CONNECTION;
}

// --- Main Loop ---
void loop() {
    if (!client || !client.connected()) {
        if (client) {
            Serial.println("Client disconnected.");
            client.stop();
            isStreamingAudio = false; // Stop streaming tasks if client disconnects
        }
        currentState = AWAITING_CONNECTION;
        client = tcpServer.available();
        if (client) {
            Serial.printf("New client connected: %s. Heap: %u\n", client.remoteIP().toString().c_str(), ESP.getFreeHeap());
            Serial.println("--> State: AWAITING_COMMAND");
            currentState = AWAITING_COMMAND;
        } else {
            vTaskDelay(pdMS_TO_TICKS(10)); // Use vTaskDelay in main loop now
            return;
        }
    }

    switch (currentState) {
        case AWAITING_COMMAND: {
            isStreamingAudio = false; // Ensure tasks are paused
            if (client.available() > 0) {
                uint8_t command = client.read();
                clearBuffers();
                if (command == CMD_REQUEST_IMU) {
                    currentState = STREAMING_IMU;
                } else if (command == CMD_REQUEST_AUDIO) {
                    xQueueReset(audioBufferQueue); // Empty the queue before starting
                    isStreamingAudio = true; // Enable the audio tasks
                    currentState = STREAMING_AUDIO;
                    Serial.println("Audio streaming started...");
                } else {
                    client.stop();
                    currentState = AWAITING_CONNECTION;
                }
            }
            break;
        }

        case STREAMING_IMU: {
            if (imuDataReady) {
                imuDataReady = false;
                sensors_event_t accel, gyro, temp;
                sox.getEvent(&accel, &gyro, &temp);
                if (imuBuffer.size() < imuBuffer.capacity()) {
                    ImuPacket pkt;
                    pkt.packetIndex = ++imuPacketCounter;
                    pkt.accX = accel.acceleration.x; pkt.accY = accel.acceleration.y; pkt.accZ = accel.acceleration.z;
                    pkt.gyroX = gyro.gyro.x; pkt.gyroY = gyro.gyro.y; pkt.gyroZ = gyro.gyro.z;
                    imuBuffer.push_back(pkt);
                }
            }
            if (imuBuffer.size() >= TARGET_IMU_PACKETS) {
                sendImuData();
                imuBuffer.clear();
            }
            break;
        }

        case STREAMING_AUDIO: {
            // The main loop does almost nothing here for audio!
            // The background tasks are doing all the work.
            // We just need to check for a new command to stop streaming.
            if (client.available() > 0) {
                 Serial.println("Command received, stopping audio stream.");
                 isStreamingAudio = false;
                 currentState = AWAITING_COMMAND;
            }
            // A small delay to prevent this loop from starving other tasks
            vTaskDelay(pdMS_TO_TICKS(10)); 
            break;
        }
        case AWAITING_CONNECTION:
        default:
            vTaskDelay(pdMS_TO_TICKS(10));
            break;
    }
}

// --- Utility Functions ---

void clearBuffers() {
    imuBuffer.clear();
    xQueueReset(audioBufferQueue); // Clear the queue instead of a vector
    imuPacketCounter = 0;
}

void sendImuData() {
    if (!client || !client.connected()) return;
    Serial.printf("Sending %zu IMU packets to client...\n", imuBuffer.size());
    for (const auto& packet : imuBuffer) {
        if (client.write((const uint8_t*)&packet, sizeof(packet)) != sizeof(packet)) {
            client.stop();
            return;
        }
    }
}

// --- Setup Functions (unchanged) ---

void setup_wifi() { 
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    for (int attempts = 0; WiFi.status() != WL_CONNECTED && attempts < 30; attempts++) {
        delay(500); Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected!");
        Serial.print("IP address: "); Serial.println(WiFi.localIP());
    } else {
        Serial.println("\nFailed to connect. Halting."); while(1) delay(1000);
    }
}

void setup_imu() {
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
    
    delay(100);
    sox.configIntOutputs(false, false);
    sox.configInt1(false, false, true, false, false);
    
    Serial.println("IMU Data Ready Interrupt Enabled on sensor.");
    delay(100);
}

void setup_microphone() {
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
    i2s_driver_install(I2S_PORT_NUM, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_PORT_NUM, &pin_config);
    i2s_zero_dma_buffer(I2S_PORT_NUM);
    Serial.println("Microphone Setup Complete.");
}