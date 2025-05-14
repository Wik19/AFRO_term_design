import socket
import struct
import time
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
from scipy.signal import butter, lfilter
from scipy.fft import fft, fftfreq
import threading

# --- Configuration ---
SERVER_IP = "192.168.11.42"
SERVER_PORT = 8080
LISTEN_DURATION_SECONDS = 5
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
# Matches ImuPacket struct in C++
# B: uint8_t type, I: uint32_t timestamp, 6f: six floats (accX..gyroZ)
IMU_PACKET_FORMAT = '<B I 6f'
IMU_PACKET_SIZE = struct.calcsize(IMU_PACKET_FORMAT)

# Matches AudioPacketHeader struct in C++
# B: uint8_t type, I: uint32_t timestamp, H: uint16_t num_samples
AUDIO_HEADER_FORMAT = '<B I H'
AUDIO_HEADER_SIZE = struct.calcsize(AUDIO_HEADER_FORMAT)
AUDIO_SAMPLE_FORMAT = '<i' # int32_t for each sample
AUDIO_SAMPLE_SIZE = struct.calcsize(AUDIO_SAMPLE_FORMAT)

# --- Data Storage ---
imu_timestamps = deque()
acc_x_data = deque()
acc_y_data = deque()
acc_z_data = deque()
gyro_x_data = deque()
gyro_y_data = deque()
gyro_z_data = deque()

audio_samples_raw = deque() # Store raw 24-bit processed samples

# --- Global flag to stop the collection thread ---
stop_collection_flag = False

def process_audio_sample(raw_int32_sample_from_network):
    """Processes the raw 32-bit sample received over network."""
    # ESP32 sends int32_t where 24-bit audio is in MSBs (data << 8)
    # Arithmetic right shift should yield the signed 24-bit value
    signed_24_bit_value = raw_int32_sample_from_network >> 8
    return signed_24_bit_value

def receive_data(sock):
    """Receives and parses data from the socket."""
    receive_buffer = bytearray()
    start_time = time.time()
    first_timestamp_millis = None
    total_bytes_received = 0
    imu_packets_received = 0
    audio_packets_received = 0

    print(f"Starting data collection for {LISTEN_DURATION_SECONDS} seconds...")

    while time.time() - start_time < LISTEN_DURATION_SECONDS and not stop_collection_flag:
        try:
            # Receive data with a small timeout to allow checking the stop flag
            chunk = sock.recv(4096) # Read up to 4KB
            if not chunk:
                print("Connection closed by server.")
                break
            receive_buffer.extend(chunk)
            total_bytes_received += len(chunk)

            # Process buffer for complete packets
            while True: # Keep processing until no complete packet is found
                if not receive_buffer:
                    break # Buffer empty

                packet_type = receive_buffer[0] # Peek at the first byte

                if packet_type == PACKET_TYPE_IMU:
                    if len(receive_buffer) >= IMU_PACKET_SIZE:
                        packet_data = receive_buffer[:IMU_PACKET_SIZE]
                        receive_buffer = receive_buffer[IMU_PACKET_SIZE:] # Consume packet

                        # Unpack IMU data
                        try:
                            _, ts, ax, ay, az, gx, gy, gz = struct.unpack(IMU_PACKET_FORMAT, packet_data)
                            if first_timestamp_millis is None:
                                first_timestamp_millis = ts
                            norm_time = (ts - first_timestamp_millis) / 1000.0

                            imu_timestamps.append(norm_time)
                            acc_x_data.append(ax)
                            acc_y_data.append(ay)
                            acc_z_data.append(az)
                            gyro_x_data.append(gx)
                            gyro_y_data.append(gy)
                            gyro_z_data.append(gz)
                            imu_packets_received += 1
                        except struct.error as e:
                            print(f"Error unpacking IMU packet: {e}")
                    else:
                        break # Need more data for a complete IMU packet

                elif packet_type == PACKET_TYPE_AUDIO:
                    if len(receive_buffer) >= AUDIO_HEADER_SIZE:
                        header_data = receive_buffer[:AUDIO_HEADER_SIZE]
                        # Unpack header first
                        try:
                            _, ts, num_samples = struct.unpack(AUDIO_HEADER_FORMAT, header_data)
                            
                            # Basic sanity check on num_samples
                            if num_samples > MIC_AUDIO_CHUNK_SAMPLES * 2: # If num_samples seems unreasonably large
                                print(f"Warning: Unusually large num_samples ({num_samples}) in audio header. Discarding byte.")
                                receive_buffer = receive_buffer[1:] # Discard and try to resync
                                continue # Skip to next iteration of inner while loop

                            samples_bytes_needed = num_samples * AUDIO_SAMPLE_SIZE
                            total_packet_size = AUDIO_HEADER_SIZE + samples_bytes_needed

                            if len(receive_buffer) >= total_packet_size:
                                # Consume header and sample data
                                sample_data_bytes = receive_buffer[AUDIO_HEADER_SIZE:total_packet_size]
                                receive_buffer = receive_buffer[total_packet_size:]

                                # Unpack samples
                                for i in range(num_samples):
                                    sample_bytes = sample_data_bytes[i*AUDIO_SAMPLE_SIZE:(i+1)*AUDIO_SAMPLE_SIZE]
                                    raw_sample_int32, = struct.unpack(AUDIO_SAMPLE_FORMAT, sample_bytes)
                                    processed_sample = process_audio_sample(raw_sample_int32)
                                    audio_samples_raw.append(processed_sample)
                                audio_packets_received += 1
                            else:
                                break # Need more data for samples

                        except struct.error as e:
                            print(f"Error unpacking Audio header: {e}")
                            receive_buffer = receive_buffer[1:]
                    else:
                         break # Need more data for header
                else:
                    # Unknown packet type - data corruption or out of sync
                    print(f"Warning: Unknown packet type {packet_type} found. Discarding byte.")
                    receive_buffer = receive_buffer[1:] # Discard the unknown byte

        except socket.timeout:
            continue # No data received in this timeout window, check stop flag/timer
        except BlockingIOError:
             time.sleep(0.001) # Socket wasn't ready, wait briefly
        except Exception as e:
            print(f"Error during socket reception: {e}")
            break # Exit loop on other errors

    print("\n--- Collection Summary ---")
    print(f"Duration: {time.time() - start_time:.2f} seconds")
    print(f"Total bytes received: {total_bytes_received}")
    print(f"IMU packets processed: {imu_packets_received}")
    print(f"Audio packets processed: {audio_packets_received} ({len(audio_samples_raw)} samples)")
    print(f"Remaining buffer size: {len(receive_buffer)}")


def butter_lowpass_filter(data, cutoff, fs, order):
    """Applies a Butterworth low-pass filter to the data."""
    if len(data) == 0: return np.array([]) # Handle empty data
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    # Ensure cutoff is valid
    if normal_cutoff >= 1.0: normal_cutoff = 0.999 # Avoid exactly 1
    if normal_cutoff <= 0.0: normal_cutoff = 0.001 # Avoid exactly 0

    try:
        b, a = butter(order, normal_cutoff, btype='low', analog=False)
        y = lfilter(b, a, data)
        return y
    except ValueError as e:
        print(f"Error creating filter (cutoff={normal_cutoff:.4f}, fs={fs}, order={order}): {e}")
        return data # Return original data on filter error


def plot_data():
    """Generates the plots based on collected data."""

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

        # 1. DC Offset Removal from Raw Audio
        mean_raw_audio = np.mean(raw_audio_np)
        audio_raw_dc_removed = raw_audio_np - mean_raw_audio
        print(f"Audio: Raw Mean (DC Offset): {mean_raw_audio:.2f}")

        plot_audio_raw = audio_raw_dc_removed
        y_label_raw_audio = 'Amplitude (24-bit ADC, DC Removed)'
        if NORMALIZE_AUDIO:
            max_abs_raw = np.max(np.abs(audio_raw_dc_removed))
            if max_abs_raw > 1e-6: # Avoid division by zero/small numbers
                plot_audio_raw = audio_raw_dc_removed / max_abs_raw
            else:
                plot_audio_raw = np.zeros_like(audio_raw_dc_removed) # Handle zero signal case
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
        audio_filtered = butter_lowpass_filter(audio_raw_dc_removed, LOWPASS_CUTOFF_HZ, I2S_SAMPLE_RATE, FILTER_ORDER)

        plot_audio_filtered = audio_filtered
        y_label_filtered_audio = 'Amplitude (Filtered, DC Removed)'
        if NORMALIZE_AUDIO:
            max_abs_filtered = np.max(np.abs(audio_filtered))
            if max_abs_filtered > 1e-6:
                plot_audio_filtered = audio_filtered / max_abs_filtered
            else:
                 plot_audio_filtered = np.zeros_like(audio_filtered)
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
        if N > 1: # Need at least 2 points for FFT
            yf = fft(audio_filtered) # Use the non-normalized filtered data for FFT magnitude
            xf = fftfreq(N, 1 / I2S_SAMPLE_RATE)[:N//2]
            fft_magnitude = 2.0/N * np.abs(yf[0:N//2])

            # Subplot 3: FFT
            axs_audio[2].plot(xf, fft_magnitude, label='FFT Magnitude', color='g')
            axs_audio[2].set_title('FFT of Filtered Audio')
            axs_audio[2].set_xlabel('Frequency (Hz)')
            axs_audio[2].set_ylabel('Magnitude')
            axs_audio[2].set_xlim(0, I2S_SAMPLE_RATE / 2)
            # axs_audio[2].set_yscale('log') # Optional log scale
            axs_audio[2].legend(loc='upper right')
            axs_audio[2].grid(True)
        else:
            axs_audio[2].set_title('Not enough data for FFT')
            axs_audio[2].grid(True)

        fig_audio.tight_layout(rect=[0, 0.03, 1, 0.95])
        print("Audio plots generated.")
    else:
        print("No audio data collected to plot.")


    # --- IMU Plotting ---
    if imu_timestamps:
        print("\nGenerating IMU plots...")
        fig_imu, axs_imu = plt.subplots(2, 1, figsize=(15, 8))
        fig_imu.canvas.manager.set_window_title('IMU Analysis')
        # Calculate approximate ODR from timestamps for title
        if len(imu_timestamps) > 1:
            avg_diff = np.mean(np.diff(list(imu_timestamps)))
            approx_odr = 1.0 / avg_diff if avg_diff > 0 else 0
        else:
            approx_odr = 0 # Or default to expected 1666
        fig_imu.suptitle(f'IMU Sensor Data (Approx ODR: {approx_odr:.0f}Hz)', fontsize=16)


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

    if audio_samples_raw or imu_timestamps:
        print("\nDisplaying plot windows (Close windows to exit script)...")
        plt.show() # Shows all generated figures
        print("Plot windows closed.")
    else:
        print("No data to display.")


if __name__ == '__main__':
    # server_ip = input("Enter the ESP32 server IP address: ") # Removed input prompt
    server_ip = SERVER_IP # Use the hardcoded IP
    if not server_ip:
        print("SERVER_IP is not set in the script. Exiting.")
        exit()

    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.settimeout(3.0) # Slightly longer timeout for connection attempt

    try:
        print(f"Connecting to {server_ip}:{SERVER_PORT}...")
        client_socket.connect((server_ip, SERVER_PORT))
        client_socket.settimeout(0.1) # Shorter timeout for recv
        print("Connected to ESP32 server.")

        # Run collection directly
        receive_data(client_socket)

    except socket.timeout:
        print(f"Connection timed out. Is the server running at {server_ip}:{SERVER_PORT}?")
        print("Check:")
        print("  - ESP32 is powered on and connected to the same WiFi network (hotspot).")
        print(f" - SERVER_IP ('{server_ip}') in the script matches the ESP32's actual IP.")
        print("  - Firewall is not blocking the connection on your PC.")
    except socket.error as e:
        print(f"Socket error: {e}")
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
        stop_collection_flag = True
    finally:
        print("Closing socket.")
        client_socket.close()

    # Plot the collected data after collection finishes or is interrupted
    plot_data()

    print("Script finished.")

