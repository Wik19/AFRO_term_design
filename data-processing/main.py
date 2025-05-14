import socket
import struct
import time
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
from scipy.signal import butter, lfilter
from scipy.fft import fft, fftfreq
import csv
import wave
from datetime import datetime

# --- Configuration ---
SERVER_IP = "192.168.11.42"
SERVER_PORT = 8080
LISTEN_DURATION_SECONDS = 2
I2S_SAMPLE_RATE = 16000
MIC_AUDIO_CHUNK_SAMPLES = 256
NORMALIZE_AUDIO = True

# Audio Filter Configuration
LOWPASS_CUTOFF_HZ = 4000.0
FILTER_ORDER = 4

# --- Packet Type Identifiers ---
PACKET_TYPE_IMU = 0x01
PACKET_TYPE_AUDIO = 0x02

# --- Struct Format Strings (Little-Endian '<') ---
IMU_PACKET_FORMAT = '<B I 6f'
IMU_PACKET_SIZE = struct.calcsize(IMU_PACKET_FORMAT)

AUDIO_HEADER_FORMAT = '<B I H'
AUDIO_HEADER_SIZE = struct.calcsize(AUDIO_HEADER_FORMAT)
AUDIO_SAMPLE_FORMAT = '<i'
AUDIO_SAMPLE_SIZE = struct.calcsize(AUDIO_SAMPLE_FORMAT)

# --- Data Storage ---
imu_timestamps = deque()
acc_x_data = deque()
acc_y_data = deque()
acc_z_data = deque()
gyro_x_data = deque()
gyro_y_data = deque()
gyro_z_data = deque()

audio_samples_raw = deque() # Stores processed 24-bit samples as Python integers

stop_collection_flag = False

def process_audio_sample(raw_int32_sample_from_network):
    # ESP32 sends int32_t where 24-bit audio is in MSBs (data << 8)
    # Arithmetic right shift should yield the signed 24-bit value
    signed_24_bit_value = raw_int32_sample_from_network >> 8
    return signed_24_bit_value

def receive_data(sock):
    receive_buffer = bytearray()
    collection_start_time = time.time()
    first_esp_timestamp_millis = None
    total_bytes_received = 0
    imu_packets_received = 0
    audio_packets_received = 0
    last_data_time = collection_start_time # Initialize last_data_time

    print(f"Starting data collection for up to {LISTEN_DURATION_SECONDS} seconds...")
    print("ESP32 will send data in batches. Expect periods of activity and short pauses.")

    sock.settimeout(1.0)

    while time.time() - collection_start_time < LISTEN_DURATION_SECONDS and not stop_collection_flag:
        chunk_received_in_iteration = False
        try:
            chunk = sock.recv(8192)
            if not chunk:
                print("Connection closed by server (or no more data in this batch cycle).")
                break
            receive_buffer.extend(chunk)
            total_bytes_received += len(chunk)
            chunk_received_in_iteration = True

            while True:
                if not receive_buffer:
                    break
                packet_type = receive_buffer[0]

                if packet_type == PACKET_TYPE_IMU:
                    if len(receive_buffer) >= IMU_PACKET_SIZE:
                        packet_data = receive_buffer[:IMU_PACKET_SIZE]
                        receive_buffer = receive_buffer[IMU_PACKET_SIZE:]
                        try:
                            _, ts_esp, ax, ay, az, gx, gy, gz = struct.unpack(IMU_PACKET_FORMAT, packet_data)
                            if first_esp_timestamp_millis is None:
                                first_esp_timestamp_millis = ts_esp
                            norm_time = (ts_esp - first_esp_timestamp_millis) / 1000.0
                            imu_timestamps.append(norm_time)
                            acc_x_data.append(ax); acc_y_data.append(ay); acc_z_data.append(az)
                            gyro_x_data.append(gx); gyro_y_data.append(gy); gyro_z_data.append(gz)
                            imu_packets_received += 1
                        except struct.error as e:
                            print(f"Error unpacking IMU packet: {e}. Buffer: {packet_data.hex()}")
                    else:
                        break
                elif packet_type == PACKET_TYPE_AUDIO:
                    if len(receive_buffer) >= AUDIO_HEADER_SIZE:
                        header_data = receive_buffer[:AUDIO_HEADER_SIZE]
                        try:
                            _, ts_esp, num_samples = struct.unpack(AUDIO_HEADER_FORMAT, header_data)
                            if num_samples > MIC_AUDIO_CHUNK_SAMPLES * 2:
                                print(f"Warning: Unusually large num_samples ({num_samples}) in audio header. Discarding byte.")
                                receive_buffer = receive_buffer[1:]
                                continue
                            samples_bytes_needed = num_samples * AUDIO_SAMPLE_SIZE
                            total_packet_size = AUDIO_HEADER_SIZE + samples_bytes_needed
                            if len(receive_buffer) >= total_packet_size:
                                sample_data_bytes = receive_buffer[AUDIO_HEADER_SIZE:total_packet_size]
                                receive_buffer = receive_buffer[total_packet_size:]
                                for i in range(num_samples):
                                    sample_bytes = sample_data_bytes[i*AUDIO_SAMPLE_SIZE:(i+1)*AUDIO_SAMPLE_SIZE]
                                    raw_sample_int32, = struct.unpack(AUDIO_SAMPLE_FORMAT, sample_bytes)
                                    processed_sample = process_audio_sample(raw_sample_int32)
                                    audio_samples_raw.append(processed_sample)
                                audio_packets_received += 1
                            else:
                                break
                        except struct.error as e:
                            print(f"Error unpacking Audio header: {e}. Buffer: {header_data.hex()}")
                    else:
                        break
                else:
                    print(f"Warning: Unknown packet type {packet_type} (0x{packet_type:02X}) found. Discarding byte to resync.")
                    receive_buffer = receive_buffer[1:]
        
        except socket.timeout:
            elapsed_time = time.time() - collection_start_time
            if total_bytes_received > 0 and time.time() - last_data_time > 3.0:
                print("\nNo data received for 3 seconds, assuming ESP32 finished sending batches for this connection cycle.")
                break
            if elapsed_time >= LISTEN_DURATION_SECONDS:
                 print("\nListen duration elapsed.")
                 break
            # print(".", end="", flush=True) # Indicate waiting
            continue
        
        except BlockingIOError:
             time.sleep(0.001)
        except Exception as e:
            print(f"Error during socket reception: {e}")
            break
        
        if chunk_received_in_iteration:
             last_data_time = time.time() # Update time of last successful data reception

    actual_duration = time.time() - collection_start_time
    print("\n--- Collection Summary ---")
    print(f"Listened for: {actual_duration:.2f} seconds (target: {LISTEN_DURATION_SECONDS}s)")
    print(f"Total bytes received: {total_bytes_received}")
    print(f"IMU packets processed: {imu_packets_received}")
    print(f"Audio chunks processed: {audio_packets_received} ({len(audio_samples_raw)} total audio samples)")
    print(f"Remaining buffer size: {len(receive_buffer)}")
    if len(receive_buffer) > 0:
        print(f"Warning: {len(receive_buffer)} bytes remaining in receive_buffer. May indicate partial packet(s).")


def butter_lowpass_filter(data, cutoff, fs, order):
    if len(data) == 0: return np.array([])
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    if normal_cutoff >= 1.0: normal_cutoff = 0.999
    if normal_cutoff <= 0.0: normal_cutoff = 0.001
    try:
        b, a = butter(order, normal_cutoff, btype='low', analog=False)
        y = lfilter(b, a, data)
        return y
    except ValueError as e:
        print(f"Error creating filter (cutoff={normal_cutoff:.4f}, fs={fs}, order={order}): {e}")
        return np.array(data)


def plot_data():
    if not audio_samples_raw and not imu_timestamps:
        print("No valid data collected for plotting.")
        return

    # --- Audio Plotting ---
    if audio_samples_raw:
        print("\nGenerating Audio plots...")
        fig_audio, axs_audio = plt.subplots(3, 1, figsize=(15, 10))
        fig_audio.canvas.manager.set_window_title('Audio Analysis')
        fig_audio.suptitle(f'Microphone Audio Analysis (Fs={I2S_SAMPLE_RATE}Hz)', fontsize=16)

        raw_audio_np = np.array(list(audio_samples_raw), dtype=np.float32)
        mean_raw_audio = np.mean(raw_audio_np) if len(raw_audio_np) > 0 else 0.0
        audio_raw_dc_removed = raw_audio_np - mean_raw_audio
        # print(f"Audio: Raw Mean (DC Offset): {mean_raw_audio:.2f}") # Less verbose

        plot_audio_raw = audio_raw_dc_removed
        y_label_raw_audio = 'Amplitude (24-bit ADC, DC Removed)'
        if NORMALIZE_AUDIO:
            max_abs_raw = np.max(np.abs(audio_raw_dc_removed)) if len(audio_raw_dc_removed) > 0 else 0
            if max_abs_raw > 1e-6:
                plot_audio_raw = audio_raw_dc_removed / max_abs_raw
            else:
                plot_audio_raw = np.zeros_like(audio_raw_dc_removed)
            y_label_raw_audio = 'Normalized Amplitude [-1, 1]'

        audio_time_axis = np.arange(len(plot_audio_raw)) / I2S_SAMPLE_RATE
        axs_audio[0].plot(audio_time_axis, plot_audio_raw, label='Raw Audio Signal', color='c', linewidth=0.5)
        axs_audio[0].set_title('Raw Audio (DC Removed & Optionally Normalized)')
        axs_audio[0].set_xlabel('Time (s)')
        axs_audio[0].set_ylabel(y_label_raw_audio)
        axs_audio[0].legend(loc='upper right'); axs_audio[0].grid(True)
        if NORMALIZE_AUDIO: axs_audio[0].set_ylim([-1.1, 1.1])

        # print(f"Applying Low-Pass Filter: Cutoff={LOWPASS_CUTOFF_HZ}Hz, Order={FILTER_ORDER}")
        audio_filtered = butter_lowpass_filter(audio_raw_dc_removed, LOWPASS_CUTOFF_HZ, I2S_SAMPLE_RATE, FILTER_ORDER)

        plot_audio_filtered = audio_filtered
        y_label_filtered_audio = 'Amplitude (Filtered, DC Removed)'
        if NORMALIZE_AUDIO:
            max_abs_filtered = np.max(np.abs(audio_filtered)) if len(audio_filtered) > 0 else 0
            if max_abs_filtered > 1e-6:
                plot_audio_filtered = audio_filtered / max_abs_filtered
            else:
                 plot_audio_filtered = np.zeros_like(audio_filtered)
            y_label_filtered_audio = 'Normalized Amplitude (Filtered) [-1, 1]'
        
        filtered_time_axis = np.arange(len(plot_audio_filtered)) / I2S_SAMPLE_RATE
        axs_audio[1].plot(filtered_time_axis, plot_audio_filtered, label=f'Low-Pass Filtered ({LOWPASS_CUTOFF_HZ}Hz)', color='m', linewidth=0.5)
        axs_audio[1].set_title('Low-Pass Filtered Audio')
        axs_audio[1].set_xlabel('Time (s)')
        axs_audio[1].set_ylabel(y_label_filtered_audio)
        axs_audio[1].legend(loc='upper right'); axs_audio[1].grid(True)
        if NORMALIZE_AUDIO: axs_audio[1].set_ylim([-1.1, 1.1])

        N_fft = len(audio_filtered) # Use filtered data for FFT
        if N_fft > 1:
            # Use the non-normalized filtered data for FFT magnitude to reflect actual signal strength
            yf = fft(audio_filtered) 
            xf = fftfreq(N_fft, 1 / I2S_SAMPLE_RATE)[:N_fft//2]
            fft_magnitude = 2.0/N_fft * np.abs(yf[0:N_fft//2])
            axs_audio[2].plot(xf, fft_magnitude, label='FFT Magnitude', color='g')
            axs_audio[2].set_title('FFT of Filtered Audio')
            axs_audio[2].set_xlabel('Frequency (Hz)'); axs_audio[2].set_ylabel('Magnitude')
            axs_audio[2].set_xlim(0, min(I2S_SAMPLE_RATE / 2, LOWPASS_CUTOFF_HZ * 1.5)) # Limit FFT plot to relevant range
            axs_audio[2].legend(loc='upper right'); axs_audio[2].grid(True)
        else:
            axs_audio[2].set_title('Not enough data for FFT'); axs_audio[2].grid(True)
        fig_audio.tight_layout(rect=[0, 0.03, 1, 0.95]); print("Audio plots generated.")
    else:
        print("No audio data collected to plot.")

    # --- IMU Plotting ---
    if imu_timestamps:
        print("\nGenerating IMU plots...")
        fig_imu, axs_imu = plt.subplots(2, 1, figsize=(15, 8))
        fig_imu.canvas.manager.set_window_title('IMU Analysis')
        
        imu_ts_list = list(imu_timestamps)
        if len(imu_ts_list) > 1:
            avg_diff = np.mean(np.diff(imu_ts_list))
            approx_odr = 1.0 / avg_diff if avg_diff > 0 else 0
        else:
            approx_odr = 0
        fig_imu.suptitle(f'IMU Sensor Data (Approx ODR from data: {approx_odr:.0f}Hz)', fontsize=16)

        axs_imu[0].plot(imu_ts_list, list(acc_x_data), label='Acc X', color='r', marker='.', linestyle='-', markersize=2)
        axs_imu[0].plot(imu_ts_list, list(acc_y_data), label='Acc Y', color='g', marker='.', linestyle='-', markersize=2)
        axs_imu[0].plot(imu_ts_list, list(acc_z_data), label='Acc Z', color='b', marker='.', linestyle='-', markersize=2)
        axs_imu[0].set_title('Accelerometer Data'); axs_imu[0].set_xlabel('Time (s, relative to ESP32 data start)')
        axs_imu[0].set_ylabel('Acceleration (m/s^2)'); axs_imu[0].legend(loc='upper right'); axs_imu[0].grid(True)

        axs_imu[1].plot(imu_ts_list, list(gyro_x_data), label='Gyro X', color='r', marker='.', linestyle='-', markersize=2)
        axs_imu[1].plot(imu_ts_list, list(gyro_y_data), label='Gyro Y', color='g', marker='.', linestyle='-', markersize=2)
        axs_imu[1].plot(imu_ts_list, list(gyro_z_data), label='Gyro Z', color='b', marker='.', linestyle='-', markersize=2)
        axs_imu[1].set_title('Gyroscope Data'); axs_imu[1].set_xlabel('Time (s, relative to ESP32 data start)')
        axs_imu[1].set_ylabel('Angular Velocity (rad/s)'); axs_imu[1].legend(loc='upper right'); axs_imu[1].grid(True)
        
        fig_imu.tight_layout(rect=[0, 0.03, 1, 0.95]); print("IMU plots generated.")
    else:
        print("No IMU data collected to plot.")

    if audio_samples_raw or imu_timestamps:
        print("\nDisplaying plot windows (Close windows to exit script)...")
        plt.show()
        print("Plot windows closed.")
    else:
        print("No data to display.")


def save_data_to_files():
    """Saves collected IMU data to CSV and audio data to WAV."""
    if not audio_samples_raw and not imu_timestamps:
        print("No data to save.")
        return

    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")

    # --- Save IMU Data to CSV ---
    if imu_timestamps:
        imu_filename = f"sensor_data/imu_data_{timestamp_str}.csv"
        print(f"\nSaving IMU data to {imu_filename}...")
        try:
            with open(imu_filename, 'w', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                # Write header
                csv_writer.writerow([
                    "Timestamp_s",
                    "AccX_m/s^2", "AccY_m/s^2", "AccZ_m/s^2",
                    "GyroX_rad/s", "GyroY_rad/s", "GyroZ_rad/s"
                ])
                # Write data rows
                # Convert deques to lists for consistent iteration if needed, though direct iteration is fine
                ts_list = list(imu_timestamps)
                ax_list = list(acc_x_data)
                ay_list = list(acc_y_data)
                az_list = list(acc_z_data)
                gx_list = list(gyro_x_data)
                gy_list = list(gyro_y_data)
                gz_list = list(gyro_z_data)

                for i in range(len(ts_list)):
                    csv_writer.writerow([
                        f"{ts_list[i]:.6f}",
                        f"{ax_list[i]:.6f}", f"{ay_list[i]:.6f}", f"{az_list[i]:.6f}",
                        f"{gx_list[i]:.6f}", f"{gy_list[i]:.6f}", f"{gz_list[i]:.6f}"
                    ])
            print(f"IMU data successfully saved to {imu_filename}")
        except IOError as e:
            print(f"Error saving IMU data to CSV: {e}")
        except Exception as e:
            print(f"An unexpected error occurred while saving IMU data: {e}")

    # --- Save Audio Data to WAV ---
    if audio_samples_raw:
        audio_filename = f"sensor_data/audio_data_{timestamp_str}.wav"
        print(f"\nSaving audio data to {audio_filename}...")
        
        # The audio_samples_raw deque contains integers representing signed 24-bit values.
        # We need to pack these into 3-byte little-endian format for the WAV file.
        # Max 24-bit signed value is 2^23 - 1 = 8388607
        # Min 24-bit signed value is -2^23 = -8388608
        
        # Ensure samples are within 24-bit range before packing
        # This step might be redundant if process_audio_sample already guarantees this.
        # Clamping to be safe:
        clamped_samples = []
        for s in audio_samples_raw:
            if s > 8388607: s = 8388607
            if s < -8388608: s = -8388608
            clamped_samples.append(s)

        try:
            with wave.open(audio_filename, 'w') as wf:
                wf.setnchannels(1)  # Mono
                wf.setsampwidth(3)  # 3 bytes for 24-bit audio
                wf.setframerate(I2S_SAMPLE_RATE)
                
                # Pack samples into 3-byte little-endian strings
                packed_frames = bytearray()
                for sample_int in clamped_samples: # Use clamped_samples
                    # struct.pack('<i', sample_int) would give 4 bytes.
                    # We need to manually take the 3 relevant bytes for 24-bit.
                    # For little-endian, these are the first 3 bytes of the 4-byte int.
                    # If sample_int is already a true 24-bit value, (sample_int & 0xFFFFFF).to_bytes(3, 'little', signed=True)
                    # The `process_audio_sample` already gives us the correct signed 24-bit integer value.
                    # We need to convert this integer to its 3-byte two's complement representation.
                    packed_frames.extend(sample_int.to_bytes(3, byteorder='little', signed=True))
                
                wf.writeframes(packed_frames)
            print(f"Audio data successfully saved to {audio_filename}")
        except wave.Error as e:
            print(f"Error saving audio data to WAV (wave.Error): {e}")
        except IOError as e:
            print(f"Error saving audio data to WAV (IOError): {e}")
        except Exception as e:
            print(f"An unexpected error occurred while saving audio data: {e}")


if __name__ == '__main__':
    server_ip = SERVER_IP
    if not server_ip:
        print("SERVER_IP is not set in the script. Exiting.")
        exit()

    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    try:
        print(f"Connecting to {server_ip}:{SERVER_PORT}...")
        client_socket.connect((server_ip, SERVER_PORT))
        print("Connected to ESP32 server.")
        receive_data(client_socket)

    except socket.timeout:
        print(f"Connection timed out. Is the server running at {server_ip}:{SERVER_PORT}?")
        print("Check:")
        print("  - ESP32 is powered on and connected to the same WiFi network.")
        print(f"  - SERVER_IP ('{server_ip}') in the script matches the ESP32's actual IP.")
        print("  - Firewall is not blocking the connection on your PC.")
    except socket.error as e:
        print(f"Socket error: {e}")
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
        stop_collection_flag = True
    finally:
        print("Closing socket.")
        client_socket.close()

    # Plot the collected data
    plot_data()

    # Save the collected data
    save_data_to_files()

    print("Script finished.")