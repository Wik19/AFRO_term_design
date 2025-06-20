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
import os
import sys 

# --- Configuration ---
SERVER_IP = "192.168.137.47"  # IMPORTANT: Make sure this is the correct IP of your ESP32
SERVER_PORT = 8080
# LISTEN_DURATION_SECONDS is now set from the command line
I2S_SAMPLE_RATE = 16000
IMU_ODR_HZ = 416.0
MIC_AUDIO_CHUNK_SAMPLES = 256
NORMALIZE_AUDIO = True

# Audio Filter Configuration
LOWPASS_CUTOFF_HZ = 300.0
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
acc_x_data, acc_y_data, acc_z_data = deque(), deque(), deque()
gyro_x_data, gyro_y_data, gyro_z_data = deque(), deque(), deque()
audio_samples_raw = deque()

stop_collection_flag = False

# --- Command definitions to send to the server ---
CMD_REQUEST_IMU = b'\x11'
CMD_REQUEST_AUDIO = b'\x22'


def process_audio_sample(raw_int32_sample_from_network):
    # ESP32 sends int32_t where 24-bit audio is in MSBs (data << 8)
    # Arithmetic right shift should yield the signed 24-bit value
    signed_24_bit_value = raw_int32_sample_from_network >> 8
    return signed_24_bit_value

# <<< CHANGED >>> Function now accepts duration as a parameter
def receive_data(sock, listen_duration):
    receive_buffer = bytearray()
    collection_start_time = time.time()
    total_bytes_received = 0
    imu_packets_received = 0
    audio_packets_received = 0

    print(f"Starting data collection for up to {listen_duration} seconds...")
    print("ESP32 will send data in batches. Expect periods of activity.")
    sock.settimeout(2.0)

    # <<< CHANGED >>> Uses the passed-in listen_duration
    while time.time() - collection_start_time < listen_duration and not stop_collection_flag:
        try:
            chunk = sock.recv(8192)
            if not chunk:
                print("Connection closed by server.")
                break
            
            receive_buffer.extend(chunk)
            total_bytes_received += len(chunk)

            while True:
                if not receive_buffer:
                    break
                
                if len(receive_buffer) < 1:
                    break
                    
                packet_type = receive_buffer[0]

                if packet_type == PACKET_TYPE_IMU and len(receive_buffer) >= IMU_PACKET_SIZE:
                    packet_data = receive_buffer[:IMU_PACKET_SIZE]
                    receive_buffer = receive_buffer[IMU_PACKET_SIZE:]
                    
                    _, packet_index, ax, ay, az, gx, gy, gz = struct.unpack(IMU_PACKET_FORMAT, packet_data)
                    
                    time_s = (packet_index - 1) / IMU_ODR_HZ
                    
                    imu_timestamps.append(time_s)
                    acc_x_data.append(ax); acc_y_data.append(ay); acc_z_data.append(az)
                    gyro_x_data.append(gx); gyro_y_data.append(gy); gyro_z_data.append(gz)
                    imu_packets_received += 1
                
                elif packet_type == PACKET_TYPE_AUDIO and len(receive_buffer) >= AUDIO_HEADER_SIZE:
                    _, ts_esp, num_samples = struct.unpack(AUDIO_HEADER_FORMAT, receive_buffer[:AUDIO_HEADER_SIZE])
                    total_packet_size = AUDIO_HEADER_SIZE + (num_samples * AUDIO_SAMPLE_SIZE)

                    if len(receive_buffer) >= total_packet_size:
                        packet_data = receive_buffer[:total_packet_size]
                        receive_buffer = receive_buffer[total_packet_size:]
                        
                        sample_data_bytes = packet_data[AUDIO_HEADER_SIZE:]
                        for i in range(num_samples):
                            sample_bytes = sample_data_bytes[i*AUDIO_SAMPLE_SIZE:(i+1)*AUDIO_SAMPLE_SIZE]
                            raw_sample_int32, = struct.unpack(AUDIO_SAMPLE_FORMAT, sample_bytes)
                            processed_sample = process_audio_sample(raw_sample_int32)
                            audio_samples_raw.append(processed_sample)
                        audio_packets_received += 1
                    else:
                        break 
                
                elif packet_type not in [PACKET_TYPE_IMU, PACKET_TYPE_AUDIO]:
                    print(f"Warning: Unknown packet type {packet_type} (0x{packet_type:02X}) found. Discarding byte to resync.")
                    receive_buffer = receive_buffer[1:]
                else:
                    break

        except socket.timeout:
            print(".", end="", flush=True)
            continue
        except Exception as e:
            print(f"\nAn error occurred during reception: {e}")
            break

    actual_duration = time.time() - collection_start_time
    print("\n--- Collection Summary ---")
    print(f"Listened for: {actual_duration:.2f} seconds")
    print(f"Total bytes received: {total_bytes_received}")
    print(f"IMU packets processed: {imu_packets_received}")
    print(f"Audio chunks processed: {audio_packets_received} ({len(audio_samples_raw)} total samples)")


def butter_lowpass_filter(data, cutoff, fs, order):
    if len(data) == 0: return np.array([])
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    if normal_cutoff >= 1.0: normal_cutoff = 0.999
    try:
        b, a = butter(order, normal_cutoff, btype='low', analog=False)
        y = lfilter(b, a, data)
        return y
    except ValueError as e:
        print(f"Error creating filter (cutoff={normal_cutoff:.4f}): {e}")
        return np.array(data)


def plot_data():
    if not audio_samples_raw and not imu_timestamps:
        print("No valid data collected for plotting.")
        return

    # --- Audio Plotting ---
    if audio_samples_raw:
        print("\nGenerating Audio plots...")
        fig_audio, axs_audio = plt.subplots(3, 1, figsize=(15, 10))
        fig_audio.suptitle(f'Microphone Audio Analysis (Fs={I2S_SAMPLE_RATE}Hz)', fontsize=16)

        raw_audio_np = np.array(list(audio_samples_raw), dtype=np.float32)
        audio_raw_dc_removed = raw_audio_np - np.mean(raw_audio_np)
        
        plot_audio_raw = audio_raw_dc_removed
        if NORMALIZE_AUDIO and np.max(np.abs(plot_audio_raw)) > 1e-6:
            plot_audio_raw /= np.max(np.abs(plot_audio_raw))

        audio_time_axis = np.arange(len(plot_audio_raw)) / I2S_SAMPLE_RATE
        axs_audio[0].plot(audio_time_axis, plot_audio_raw, linewidth=0.5)
        axs_audio[0].set_title('Raw Audio (DC Removed & Normalized)')
        axs_audio[0].set_xlabel('Time (s)'); axs_audio[0].set_ylabel('Amplitude')
        axs_audio[0].grid(True)

        audio_filtered = butter_lowpass_filter(audio_raw_dc_removed, LOWPASS_CUTOFF_HZ, I2S_SAMPLE_RATE, FILTER_ORDER)
        if NORMALIZE_AUDIO and np.max(np.abs(audio_filtered)) > 1e-6:
            audio_filtered /= np.max(np.abs(audio_filtered))

        axs_audio[1].plot(audio_time_axis, audio_filtered, color='m', linewidth=0.5)
        axs_audio[1].set_title(f'Low-Pass Filtered Audio ({LOWPASS_CUTOFF_HZ}Hz)')
        axs_audio[1].set_xlabel('Time (s)'); axs_audio[1].set_ylabel('Amplitude')
        axs_audio[1].grid(True)

        N_fft = len(audio_filtered)
        if N_fft > 1:
            yf = fft(audio_filtered)
            xf = fftfreq(N_fft, 1 / I2S_SAMPLE_RATE)[:N_fft//2]
            fft_magnitude = 2.0/N_fft * np.abs(yf[0:N_fft//2])
            axs_audio[2].plot(xf, fft_magnitude, color='g')
            axs_audio[2].set_title('FFT of Filtered Audio')
            axs_audio[2].set_xlabel('Frequency (Hz)'); axs_audio[2].set_ylabel('Magnitude')
            axs_audio[2].set_xlim(0, LOWPASS_CUTOFF_HZ * 1.2); axs_audio[2].grid(True)
        fig_audio.tight_layout(rect=[0, 0.03, 1, 0.95])

    # --- IMU Plotting ---
    if imu_timestamps:
        print("\nGenerating IMU plots...")
        fig_imu, axs_imu = plt.subplots(2, 1, figsize=(15, 8), sharex=True)
        approx_odr = 1.0 / np.mean(np.diff(list(imu_timestamps))) if len(imu_timestamps) > 1 else 0
        fig_imu.suptitle(f'IMU Sensor Data (Approx. ODR: {approx_odr:.0f}Hz)', fontsize=16)

        axs_imu[0].plot(list(imu_timestamps), list(acc_x_data), label='Acc X', marker='.', linestyle='-', markersize=2)
        axs_imu[0].plot(list(imu_timestamps), list(acc_y_data), label='Acc Y', marker='.', linestyle='-', markersize=2)
        axs_imu[0].plot(list(imu_timestamps), list(acc_z_data), label='Acc Z', marker='.', linestyle='-', markersize=2)
        axs_imu[0].set_title('Accelerometer'); axs_imu[0].set_ylabel('m/s^2'); axs_imu[0].legend(); axs_imu[0].grid(True)
        
        axs_imu[1].plot(list(imu_timestamps), list(gyro_x_data), label='Gyro X', marker='.', linestyle='-', markersize=2)
        axs_imu[1].plot(list(imu_timestamps), list(gyro_y_data), label='Gyro Y', marker='.', linestyle='-', markersize=2)
        axs_imu[1].plot(list(imu_timestamps), list(gyro_z_data), label='Gyro Z', marker='.', linestyle='-', markersize=2)
        axs_imu[1].set_title('Gyroscope'); axs_imu[1].set_xlabel('Time (s)'); axs_imu[1].set_ylabel('rad/s'); axs_imu[1].legend(); axs_imu[1].grid(True)
        
        fig_imu.tight_layout(rect=[0, 0.03, 1, 0.95])

    if audio_samples_raw or imu_timestamps:
        plt.show()


def save_data_to_files():
    if not audio_samples_raw and not imu_timestamps:
        print("No data to save.")
        return

    if not os.path.exists("sensor_data"):
        os.makedirs("sensor_data")
        
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")

    # --- Save IMU Data to CSV ---
    if imu_timestamps:
        imu_filename = f"sensor_data/imu_data_{timestamp_str}.csv"
        print(f"\nSaving IMU data to {imu_filename}...")
        with open(imu_filename, 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(["Timestamp_s", "AccX_m/s^2", "AccY_m/s^2", "AccZ_m/s^2", "GyroX_rad/s", "GyroY_rad/s", "GyroZ_rad/s"])
            for i in range(len(imu_timestamps)):
                csv_writer.writerow([
                    f"{list(imu_timestamps)[i]:.6f}", f"{list(acc_x_data)[i]:.6f}", f"{list(acc_y_data)[i]:.6f}", f"{list(acc_z_data)[i]:.6f}",
                    f"{list(gyro_x_data)[i]:.6f}", f"{list(gyro_y_data)[i]:.6f}", f"{list(gyro_z_data)[i]:.6f}"
                ])
        print("IMU data saved.")

    # --- Save Audio Data to WAV ---
    if audio_samples_raw:
        audio_filename = f"sensor_data/audio_data_{timestamp_str}.wav"
        print(f"\nSaving audio data to {audio_filename}...")
        
        packed_frames = bytearray()
        for s in audio_samples_raw:
            s = max(min(s, 8388607), -8388608) # Clamp to 24-bit signed range
            packed_frames.extend(s.to_bytes(3, byteorder='little', signed=True))

        with wave.open(audio_filename, 'w') as wf:
            wf.setnchannels(1)
            wf.setsampwidth(3)
            wf.setframerate(I2S_SAMPLE_RATE)
            wf.writeframes(packed_frames)
        print("Audio data saved.")


# <<< CHANGED >>> Main execution block now parses command-line arguments
if __name__ == '__main__':
    # Check if the correct number of arguments is provided
    if len(sys.argv) != 3:
        print("Usage: python TermDesign.py <type> <duration>")
        print("  <type>: 'imu' or 'audio'")
        print("  <duration>: listening time in seconds (e.g., 10)")
        sys.exit(1)

    # Parse arguments
    data_type_arg = sys.argv[1].lower()
    duration_arg_str = sys.argv[2]

    # Validate data type
    if data_type_arg == 'imu':
        request_cmd = CMD_REQUEST_IMU
    elif data_type_arg == 'audio':
        request_cmd = CMD_REQUEST_AUDIO
    else:
        print(f"Error: Invalid data type '{data_type_arg}'. Choose 'imu' or 'audio'.")
        sys.exit(1)

    # Validate duration
    try:
        listen_duration = int(duration_arg_str)
        if listen_duration <= 0:
            print("Error: Duration must be a positive number.")
            sys.exit(1)
    except ValueError:
        print(f"Error: Invalid duration '{duration_arg_str}'. Please provide a number.")
        sys.exit(1)

    # --- Connection and Execution ---
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        print(f"Connecting to {SERVER_IP}:{SERVER_PORT}...")
        client_socket.connect((SERVER_IP, SERVER_PORT))
        print("Connected to ESP32 server.")

        print(f"Requesting '{data_type_arg}' data for {listen_duration} seconds...")
        client_socket.sendall(request_cmd)

        # Pass the parsed duration to the receive function
        receive_data(client_socket, listen_duration)

    except socket.timeout:
        print(f"\nConnection timed out. Check that the ESP32 is running and the IP '{SERVER_IP}' is correct.")
    except socket.error as e:
        print(f"\nSocket error: {e}")
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        print("Closing socket.")
        client_socket.close()

    plot_data()
    save_data_to_files()

    print("Script finished.")