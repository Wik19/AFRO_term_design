#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include "driver/i2s.h"

// IMU (LSM6DSO32) SPI Pins
#define IMU_CS_PIN 4
#define IMU_SCK_PIN 6
#define IMU_MISO_PIN 2
#define IMU_MOSI_PIN 5

Adafruit_LSM6DSOX sox;

// Microphone (INMP441) I2S Pins
#define I2S_MIC_SERIAL_CLOCK_PIN 10 // SCK
#define I2S_MIC_WORD_SELECT_PIN 9   // WS / LRCL
#define I2S_MIC_SERIAL_DATA_PIN 8   // SD / DOUT

// I2S Configuration
#define I2S_PORT_NUM I2S_NUM_0
#define I2S_SAMPLE_RATE 16000
#define I2S_READ_BUFFER_SIZE_BYTES (1024 * 2) // Buffer to hold I2S data (e.g., 1024 samples * 2 bytes for 16-bit, or 4 bytes for 32-bit)
                                          // INMP441 is 24-bit, ESP32 I2S reads it often as 32-bit samples (left-justified)
#define I2S_BITS_PER_SAMPLE I2S_BITS_PER_SAMPLE_32BIT // Read as 32-bit samples
#define NUM_MIC_SAMPLES_TO_PRINT_PER_BLOCK 256 // How many mic samples to print in one go

int32_t i2s_read_buffer[NUM_MIC_SAMPLES_TO_PRINT_PER_BLOCK]; // Buffer to hold samples after reading from DMA

void setup_imu() {
  Serial.println("Setting up IMU (LSM6DSO32)...");

  // Initialize SPI for the IMU manually, as MOSI might be non-default for the global SPI object
  // The CS pin is not part of SPI.begin() as it's toggled manually by the library
  SPI.begin(IMU_SCK_PIN, IMU_MISO_PIN, IMU_MOSI_PIN, -1); 

  if (!sox.begin_SPI(IMU_CS_PIN, &SPI)) {
    Serial.println("Failed to find LSM6DSOX chip via SPI. Check wiring.");
    while (1) delay(10);
  }
  Serial.println("LSM6DSOX Found!");

  // Set Accelerometer Range
  sox.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
  Serial.print("Accelerometer range set to: ");
  switch (sox.getAccelRange()) {
    case LSM6DS_ACCEL_RANGE_2_G: Serial.println("+-2G"); break;
    case LSM6DS_ACCEL_RANGE_4_G: Serial.println("+-4G"); break;
    case LSM6DS_ACCEL_RANGE_8_G: Serial.println("+-8G"); break;
    case LSM6DS_ACCEL_RANGE_16_G: Serial.println("+-16G"); break;
  }

  // Set Gyroscope Range
  sox.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
  Serial.print("Gyro range set to: ");
  switch (sox.getGyroRange()) {
    case LSM6DS_GYRO_RANGE_125_DPS: Serial.println("125 DPS"); break;
    case LSM6DS_GYRO_RANGE_250_DPS: Serial.println("250 DPS"); break;
    case LSM6DS_GYRO_RANGE_500_DPS: Serial.println("500 DPS"); break;
    case LSM6DS_GYRO_RANGE_1000_DPS: Serial.println("1000 DPS"); break;
    case LSM6DS_GYRO_RANGE_2000_DPS: Serial.println("2000 DPS"); break;
  }

  // Set Output Data Rate (ODR)
  sox.setAccelDataRate(LSM6DS_RATE_104_HZ);
  sox.setGyroDataRate(LSM6DS_RATE_104_HZ);
  Serial.print("Accelerometer ODR set to: ");
  switch (sox.getAccelDataRate()) {
    case LSM6DS_RATE_SHUTDOWN: Serial.println("Shutdown"); break;
    case LSM6DS_RATE_12_5_HZ: Serial.println("12.5 Hz"); break;
    case LSM6DS_RATE_26_HZ: Serial.println("26 Hz"); break;
    case LSM6DS_RATE_52_HZ: Serial.println("52 Hz"); break;
    case LSM6DS_RATE_104_HZ: Serial.println("104 Hz"); break;
    // ... add other rates as needed
    default: Serial.println("Other"); break;
  }
   Serial.print("Gyro ODR set to: ");
  switch (sox.getGyroDataRate()) {
    case LSM6DS_RATE_SHUTDOWN: Serial.println("Shutdown"); break;
    case LSM6DS_RATE_12_5_HZ: Serial.println("12.5 Hz"); break;
    // ... add other rates
    case LSM6DS_RATE_104_HZ: Serial.println("104 Hz"); break;
    default: Serial.println("Other"); break;
  }
  Serial.println("IMU Setup Complete.");
}

void setup_microphone() {
  Serial.println("Setting up Microphone (INMP441)...");
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = I2S_SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE, // INMP441 is 24-bit, data is LSB padded to 32 or MSB aligned in 32-bit word
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, // Assuming L/R pin of INMP441 is LOW for Left channel
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4, // Number of DMA buffers
    .dma_buf_len = 1024, // Length of each DMA buffer in samples
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_MIC_SERIAL_CLOCK_PIN,
    .ws_io_num = I2S_MIC_WORD_SELECT_PIN,
    .data_out_num = I2S_PIN_NO_CHANGE, // Not used for RX
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
  
  // The INMP441 needs about 85ms (2^18 SCK cycles at 3.072MHz) to power up.
  // SCK for I2S is sample_rate * bits_per_sample_per_frame (e.g., 16000 * 32 for mono 32-bit).
  // Let's give it some time.
  Serial.println("Waiting for microphone to settle...");
  delay(200); 
  Serial.println("Microphone Setup Complete.");
}


void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // wait for serial port to connect. Needed for native USB
  }
  Serial.println("ESP32-C3 IMU and Microphone Test");

  setup_imu();
  setup_microphone();

  Serial.println("Setup finished. Starting data acquisition...");
  Serial.println("Format: TYPE,Timestamp,val1,val2,...");
}

void loop() {
  // Read IMU data
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp; // Adafruit library also reads temperature
  
  if (sox.getEvent(&accel, &gyro, &temp)) { // getEvent also checks if data is new
    Serial.print("IMU,");
    Serial.print(millis());
    Serial.print(",");
    Serial.print(accel.acceleration.x, 4); Serial.print(",");
    Serial.print(accel.acceleration.y, 4); Serial.print(",");
    Serial.print(accel.acceleration.z, 4); Serial.print(",");
    Serial.print(gyro.gyro.x, 4); Serial.print(",");
    Serial.print(gyro.gyro.y, 4); Serial.print(",");
    Serial.print(gyro.gyro.z, 4);
    // Serial.print(","); // Uncomment if you want to add temperature
    // Serial.print(temp.temperature, 2);
    Serial.println();
  } else {
    // Serial.println("Failed to get IMU event or no new data.");
  }

  // Read Microphone data
  size_t bytes_read = 0;
  // esp_err_t i2s_err = i2s_read(I2S_PORT_NUM, (void*)i2s_read_buffer, sizeof(i2s_read_buffer), &bytes_read, pdMS_TO_TICKS(100)); // Read NUM_MIC_SAMPLES_TO_PRINT_PER_BLOCK samples
  esp_err_t i2s_err = i2s_read(I2S_PORT_NUM, (void*)i2s_read_buffer, NUM_MIC_SAMPLES_TO_PRINT_PER_BLOCK * sizeof(int32_t), &bytes_read, pdMS_TO_TICKS(100));


  if (i2s_err == ESP_OK && bytes_read > 0) {
    int samples_read = bytes_read / sizeof(int32_t);
    if (samples_read > 0) {
      Serial.print("MIC,");
      Serial.print(millis());
      for (int i = 0; i < samples_read; i++) {
        // The INMP441 data is 24-bit, left-justified in the 32-bit sample.
        // To get the actual 24-bit signed value, you might need to shift if it's not sign-extended correctly by the hardware.
        // However, for raw data processing on the laptop, sending the full int32_t might be fine,
        // and then Python can handle extracting the top 24 bits.
        // The ESP32 I2S driver typically provides the 24-bit data in the most significant bits of the 32-bit sample.
        Serial.print(",");
        Serial.print(i2s_read_buffer[i]); 
      }
      Serial.println();
    }
  } else if (i2s_err != ESP_OK) {
    // Serial.printf("I2S read error: %d\n", i2s_err);
  }
  
  // Adjust delay as needed to control overall sampling/printing rate
  // Note: IMU ODR is 104Hz (~9.6ms). I2S tries to read a block which might take time.
  // A small delay helps prevent spamming if IMU data isn't ready or I2S read is too fast.
  delay(5); 
}