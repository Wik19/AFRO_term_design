import serial
import time
import matplotlib.pyplot as plt
import numpy as np
from collections import deque
from scipy.signal import butter, lfilter, freqz
from scipy.fft import fft, fftfreq

# --- Configuration ---
SERIAL_PORT = 'COM3'
BAUD_RATE = 2000000 # Ensure ESP32 sketch matches this!
# BAUD_RATE = 115200 # Fallback if 2M is unstable
LISTEN_DURATION_SECONDS = 5
I2S_SAMPLE_RATE = 16000  # Must match the ESP32 sketch
NORMALIZE_AUDIO = True # Set to True to normalize audio plot to [-1, 1]

# Audio Filter Configuration
LOWPASS_CUTOFF_HZ = 4000.0  # Cutoff frequency for the low-pass filter
FILTER_ORDER = 4         # Order of the Butterworth filter

# --- Data Storage ---
imu_timestamps = deque()
acc_x_data = deque()
acc_y_data = deque()
acc_z_data = deque()
gyro_x_data = deque()
gyro_y_data = deque()
gyro_z_data = deque()

mic_timestamps = deque() # Timestamps for the start of each audio block
audio_samples = deque()  # All collected audio samples

def process_audio_sample(raw_int32_sample_from_serial):
    """
    The ESP32 sends int32_t where the 24-bit audio data is in the MSBs.
    An arithmetic right shift by 8 should yield the correct signed 24-bit value.
    """
    signed_24_bit_value = raw_int32_sample_from_serial >> 8
    return signed_24_bit_value

def butter_lowpass_filter(data, cutoff, fs, order):
    """Applies a Butterworth low-pass filter to the data."""
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    y = lfilter(b, a, data)
    return y

def main():
    print(f"Attempting to connect to {SERIAL_PORT} at {BAUD_RATE} baud...")
    ser = None 
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1) 
        if BAUD_RATE > 115200:
             ser.set_buffer_size(rx_size=20480, tx_size=4096) 
        print(f"Connected to {SERIAL_PORT}.")
        time.sleep(1) 
        ser.reset_input_buffer()
        print("Input buffer cleared.")
    except serial.SerialException as e:
        print(f"Error: Could not open serial port {SERIAL_PORT}: {e}")
        return
    except Exception as e: 
        print(f"Error setting up serial port: {e}")
        if ser and ser.is_open:
            ser.close()
        return

    print(f"Listening for data for {LISTEN_DURATION_SECONDS} seconds...")
    start_time = time.time()
    first_timestamp_millis = None
    line_count = 0
    data_line_count = 0
    parse_error_count = 0
    
    serial_buffer = bytearray()

    try:
        while time.time() - start_time < LISTEN_DURATION_SECONDS:
            if ser.in_waiting > 0:
                try:
                    bytes_read = ser.read(ser.in_waiting)
                    serial_buffer.extend(bytes_read)

                    while b'\n' in serial_buffer:
                        line_bytes, serial_buffer = serial_buffer.split(b'\n', 1)
                        line = line_bytes.decode('utf-8', errors='ignore').strip()
                        line_count += 1

                        if not line:
                            continue

                        if line.startswith("IMU,"):
                            parts = line.split(',')
                            if len(parts) == 8: 
                                try:
                                    current_esp_millis = int(parts[1])
                                    if first_timestamp_millis is None:
                                        first_timestamp_millis = current_esp_millis
                                    
                                    normalized_time_sec = (current_esp_millis - first_timestamp_millis) / 1000.0

                                    imu_timestamps.append(normalized_time_sec)
                                    acc_x_data.append(float(parts[2]))
                                    acc_y_data.append(float(parts[3]))
                                    acc_z_data.append(float(parts[4]))
                                    gyro_x_data.append(float(parts[5]))
                                    gyro_y_data.append(float(parts[6]))
                                    gyro_z_data.append(float(parts[7]))
                                    data_line_count +=1
                                except (ValueError, IndexError) as parse_err:
                                    # print(f"Warning: Error parsing IMU data fields in line '{line[:60]}...': {parse_err}")
                                    parse_error_count += 1
                            else:
                                # print(f"Warning: Malformed IMU line (expected 8 parts, got {len(parts)}): '{line[:60]}...'")
                                parse_error_count += 1
                        
                        elif line.startswith("MIC,"):
                            parts = line.split(',')
                            if len(parts) >= 3: 
                                try:
                                    current_esp_millis = int(parts[1])
                                    if first_timestamp_millis is None:
                                        first_timestamp_millis = current_esp_millis
                                    # normalized_time_sec = (current_esp_millis - first_timestamp_millis) / 1000.0
                                    # mic_timestamps.append(normalized_time_sec) # Not strictly needed if we make one continuous audio stream
                                    
                                    block_sample_count = 0
                                    for i in range(2, len(parts)):
                                        try:
                                            raw_sample = int(parts[i])
                                            processed_sample = process_audio_sample(raw_sample)
                                            audio_samples.append(processed_sample)
                                            block_sample_count +=1
                                        except ValueError:
                                            parse_error_count += 1 
                                    if block_sample_count > 0:
                                        data_line_count +=1
                                except (ValueError, IndexError) as parse_err:
                                    # print(f"Warning: Error parsing MIC data fields in line '{line[:60]}...': {parse_err}")
                                    parse_error_count += 1
                            else:
                                # print(f"Warning: Malformed MIC line (expected at least 3 parts, got {len(parts)}): '{line[:60]}...'")
                                parse_error_count += 1
                        else:
                            pass 
                
                except Exception as e:
                    print(f"An unexpected error occurred while processing serial data: {e}")
                    time.sleep(0.01) 
            else:
                time.sleep(0.0001) 

    except KeyboardInterrupt:
        print("Listener interrupted by user.")
    finally:
        if ser and ser.is_open:
            ser.close()
            print("Serial port closed.")
        elif not ser:
            print("Serial port was not opened.")

    print(f"\n--- Collection Summary ---")
    print(f"Total lines processed from buffer: {line_count}")
    print(f"Valid data lines parsed: {data_line_count}")
    print(f"Field parsing errors: {parse_error_count}") # Includes individual sample parse errors
    print(f"Collected {len(imu_timestamps)} IMU data points.")
    print(f"Collected {len(audio_samples)} audio samples.")

    if not audio_samples and not imu_timestamps:
        print("No valid data collected for plotting. Exiting.")
        return

    # --- Audio Plotting ---
    if audio_samples:
        print("\nGenerating Audio plots...")
        fig_audio, axs_audio = plt.subplots(3, 1, figsize=(15, 10)) 
        fig_audio.suptitle(f'Microphone Audio Analysis (Fs={I2S_SAMPLE_RATE}Hz)', fontsize=16)

        raw_audio_np = np.array(list(audio_samples), dtype=np.float32)
        
        # 1. DC Offset Removal from Raw Audio
        mean_raw_audio = np.mean(raw_audio_np)
        audio_raw_dc_removed = raw_audio_np - mean_raw_audio
        print(f"Audio: Raw Mean (DC Offset): {mean_raw_audio:.2f}")

        plot_audio_raw = audio_raw_dc_removed
        y_label_raw_audio = 'Amplitude (24-bit ADC, DC Removed)'
        if NORMALIZE_AUDIO:
            max_abs_raw = np.max(np.abs(audio_raw_dc_removed))
            if max_abs_raw > 0:
                plot_audio_raw = audio_raw_dc_removed / max_abs_raw
            y_label_raw_audio = 'Normalized Amplitude [-1, 1]'
        
        audio_time_axis = np.arange(len(plot_audio_raw)) / I2S_SAMPLE_RATE

        # Subplot 1: Raw (DC-removed, optionally normalized) Audio
        axs_audio[0].plot(audio_time_axis, plot_audio_raw, label='Raw Audio Signal', color='c', linewidth=0.5)
        axs_audio[0].set_title('Raw Audio (DC Removed)')
        axs_audio[0].set_xlabel('Time (s)')
        axs_audio[0].set_ylabel(y_label_raw_audio)
        axs_audio[0].legend(loc='upper right')
        axs_audio[0].grid(True)
        if NORMALIZE_AUDIO:
            axs_audio[0].set_ylim([-1.1, 1.1])

        # 2. Low-pass Filter
        print(f"Applying Low-Pass Filter: Cutoff={LOWPASS_CUTOFF_HZ}Hz, Order={FILTER_ORDER}")
        # Use the DC-removed audio for filtering
        audio_filtered = butter_lowpass_filter(audio_raw_dc_removed, LOWPASS_CUTOFF_HZ, I2S_SAMPLE_RATE, FILTER_ORDER)
        
        plot_audio_filtered = audio_filtered
        y_label_filtered_audio = 'Amplitude (Filtered, DC Removed)'
        if NORMALIZE_AUDIO: # Re-normalize if needed, as filtering can change amplitude
            max_abs_filtered = np.max(np.abs(audio_filtered))
            if max_abs_filtered > 0:
                plot_audio_filtered = audio_filtered / max_abs_filtered
            y_label_filtered_audio = 'Normalized Amplitude (Filtered) [-1, 1]'

        # Subplot 2: Filtered Audio
        axs_audio[1].plot(audio_time_axis, plot_audio_filtered, label=f'Low-Pass Filtered ({LOWPASS_CUTOFF_HZ}Hz)', color='m', linewidth=0.5)
        axs_audio[1].set_title('Low-Pass Filtered Audio')
        axs_audio[1].set_xlabel('Time (s)')
        axs_audio[1].set_ylabel(y_label_filtered_audio)
        axs_audio[1].legend(loc='upper right')
        axs_audio[1].grid(True)
        if NORMALIZE_AUDIO:
            axs_audio[1].set_ylim([-1.1, 1.1])
            
        # 3. FFT of Filtered Audio
        N = len(audio_filtered)
        if N > 0:
            yf = fft(audio_filtered) # Use the non-normalized filtered data for FFT
            xf = fftfreq(N, 1 / I2S_SAMPLE_RATE)[:N//2] # Get frequencies up to Nyquist
            
            fft_magnitude = 2.0/N * np.abs(yf[0:N//2]) # Normalized magnitude

            # Subplot 3: FFT
            axs_audio[2].plot(xf, fft_magnitude, label='FFT Magnitude', color='g')
            axs_audio[2].set_title('FFT of Filtered Audio')
            axs_audio[2].set_xlabel('Frequency (Hz)')
            axs_audio[2].set_ylabel('Magnitude')
            axs_audio[2].set_xlim(0, I2S_SAMPLE_RATE / 2) # Show up to Nyquist frequency
            # Optional: Log scale for Y axis if magnitudes vary widely
            # axs_audio[2].set_yscale('log')
            # axs_audio[2].set_ylim(bottom=1e-3) # Adjust if using log scale
            axs_audio[2].legend(loc='upper right')
            axs_audio[2].grid(True)
        else:
            axs_audio[2].set_title('No data for FFT')

        fig_audio.tight_layout(rect=[0, 0.03, 1, 0.95])
        print("Audio plots generated.")

    else:
        print("No audio data collected to plot.")


    # --- IMU Plotting ---
    if imu_timestamps:
        print("\nGenerating IMU plots...")
        fig_imu, axs_imu = plt.subplots(2, 1, figsize=(15, 8))
        fig_imu.suptitle(f'IMU Sensor Data ({SERIAL_PORT} @ {BAUD_RATE}bps)', fontsize=16)

        # Subplot 1: Accelerometer Data
        axs_imu[0].plot(list(imu_timestamps), list(acc_x_data), label='Acc X', color='r', marker='.', linestyle='-', markersize=2)
        axs_imu[0].plot(list(imu_timestamps), list(acc_y_data), label='Acc Y', color='g', marker='.', linestyle='-', markersize=2)
        axs_imu[0].plot(list(imu_timestamps), list(acc_z_data), label='Acc Z', color='b', marker='.', linestyle='-', markersize=2)
        axs_imu[0].set_title('Accelerometer Data')
        axs_imu[0].set_xlabel('Time (s, relative to ESP32 data start)')
        axs_imu[0].set_ylabel('Acceleration (m/s^2)')
        axs_imu[0].legend(loc='upper right')
        axs_imu[0].grid(True)

        # Subplot 2: Gyroscope Data
        axs_imu[1].plot(list(imu_timestamps), list(gyro_x_data), label='Gyro X', color='r', marker='.', linestyle='-', markersize=2)
        axs_imu[1].plot(list(imu_timestamps), list(gyro_y_data), label='Gyro Y', color='g', marker='.', linestyle='-', markersize=2)
        axs_imu[1].plot(list(imu_timestamps), list(gyro_z_data), label='Gyro Z', color='b', marker='.', linestyle='-', markersize=2)
        axs_imu[1].set_title('Gyroscope Data')
        axs_imu[1].set_xlabel('Time (s, relative to ESP32 data start)')
        axs_imu[1].set_ylabel('Angular Velocity (rad/s)') 
        axs_imu[1].legend(loc='upper right')
        axs_imu[1].grid(True)
        
        fig_imu.tight_layout(rect=[0, 0.03, 1, 0.95])
        print("IMU plots generated.")
    else:
        print("No IMU data collected to plot.")

    if audio_samples or imu_timestamps:
        print("\nDisplaying plot windows...")
        plt.show() # Shows all figures
        print("Plot windows closed.")
    else:
        print("No data to display.")


if __name__ == '__main__':
    main()
