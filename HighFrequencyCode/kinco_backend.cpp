// highfrequencycode/kinco_backend.cpp
// Author: Gemini AI
// Description: High-frequency backend for reading Kinco motor position via Modbus RTU.

#include <pybind11/pybind11.h>
#include <pybind11/stl.h> // Required for string conversions & other C++ STL containers
#include <pybind11/chrono.h> // For time-related types if needed for pybind interface

#include <windows.h>     // For Windows API (serial port, threading)
#include <cstdint>       // For int32_t, uint16_t, uint8_t
#include <string>
#include <vector>
#include <thread>        // For std::thread
#include <atomic>        // For std::atomic
#include <chrono>        // For std::chrono::milliseconds
#include <stdexcept>     // For std::runtime_error
#include <iostream>      // For std::cout, std::cerr (debugging)
#include <sstream>       // For std::ostringstream
#include <iomanip>       // For std::hex, std::setw, std::setfill

namespace py = pybind11;

// --- Configuration Constants ---
const uint8_t KINCO_SLAVE_ID = 0x01;
const uint16_t KINCO_POSITION_REGISTER_START = 0x3700; // Actual position (signed 32-bit)
const uint16_t KINCO_REGISTERS_TO_READ_COUNT = 2;    // Reading two 16-bit registers for a 32-bit value
const int SERIAL_READ_TIMEOUT_MS = 50; // Timeout for serial read operations
const int SERIAL_WRITE_TIMEOUT_MS = 50; // Timeout for serial write operations
const int LOOP_SLEEP_DURATION_MS = 1; // Target ~1kHz polling

// --- Global State for C++ Backend ---
static HANDLE g_hSerial = INVALID_HANDLE_VALUE;
static std::atomic<bool> g_keep_running_thread(false);
static std::atomic<bool> g_port_initialized_successfully(false);
static std::atomic<int32_t> g_latest_motor_position(0);
static std::thread g_reader_thread;
static std::atomic<DWORD> g_last_com_error_code(0);
static std::string g_last_error_message = ""; // More descriptive error

// --- Helper Functions ---

/**
 * @brief Logs messages to Python console (if interpreter is available) or std::cout.
 * @param source Typically "[C++ Backend]"
 * @param message The message string.
 */
void cpp_log_message(const std::string& source, const std::string& message) {
    try {
        py::gil_scoped_acquire gil; // Acquire GIL for calling Python C-API
        py::print(source, message);
    } catch (const std::exception& e) {
        std::cout << source << " (py::print failed: " << e.what() << "): " << message << std::endl;
    }
}

/**
 * @brief Logs a Windows error, converting error code to a human-readable string.
 * @param function_name Name of the Windows API function that failed.
 * @param error_code The error code from GetLastError().
 */
void cpp_log_windows_error(const std::string& function_name, DWORD error_code) {
    char msg_buf[256];
    FormatMessageA(
        FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
        NULL, error_code, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
        msg_buf, sizeof(msg_buf), NULL);
    
    std::ostringstream oss;
    oss << function_name << " failed. Code: " << error_code << ". Message: " << msg_buf;
    g_last_error_message = oss.str(); // Store the full error message
    g_last_com_error_code.store(error_code, std::memory_order_relaxed);
    cpp_log_message("[C++ Error]", g_last_error_message);
}

/**
 * @brief Calculates Modbus RTU CRC16.
 * @param data Pointer to the data buffer.
 * @param length Length of the data buffer.
 * @return The calculated CRC16 value.
 */
uint16_t calculate_crc16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= static_cast<uint16_t>(data[i]);
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief Builds a Modbus RTU frame for reading holding registers.
 * @param slave_id The slave ID.
 * @param start_register The starting register address.
 * @param num_registers The number of registers to read.
 * @return A vector of bytes representing the Modbus frame.
 */
std::vector<uint8_t> build_modbus_read_frame(uint8_t slave_id, uint16_t start_register, uint16_t num_registers) {
    std::vector<uint8_t> frame(8); // SlaveID(1) + FuncCode(1) + Addr(2) + Count(2) + CRC(2)
    frame[0] = slave_id;
    frame[1] = 0x03; // Function code for Read Holding Registers
    frame[2] = (start_register >> 8) & 0xFF; // Start Address Hi
    frame[3] = start_register & 0xFF;        // Start Address Lo
    frame[4] = (num_registers >> 8) & 0xFF;  // Quantity Hi
    frame[5] = num_registers & 0xFF;         // Quantity Lo

    uint16_t crc = calculate_crc16(frame.data(), 6);
    frame[6] = crc & 0xFF;        // CRC Lo
    frame[7] = (crc >> 8) & 0xFF; // CRC Hi
    return frame;
}

/**
 * @brief The main loop for the background serial reader thread.
 *        Continuously polls the motor for its position.
 * @param port_name_com The COM port name (e.g., "COM3" or "\\.\COM11").
 * @param baud_rate The baud rate for serial communication.
 */
void serial_reader_loop(std::string port_name_com, int baud_rate) {
    g_port_initialized_successfully.store(false, std::memory_order_relaxed);
    g_last_com_error_code.store(0, std::memory_order_relaxed);
    g_last_error_message.clear();

    std::string port_name_to_open = port_name_com;
    // Adjust for COM ports >= 10 on Windows
    if (port_name_to_open.rfind("COM", 0) == 0 && port_name_to_open.length() > 4) {
        try {
            if (std::stoi(port_name_to_open.substr(3)) >= 10) {
                port_name_to_open = "\\\\.\\" + port_name_to_open;
            }
        } catch (...) { /* Ignore parsing error, use original name */ }
    }

    HANDLE hComm = CreateFileA(port_name_to_open.c_str(),
                              GENERIC_READ | GENERIC_WRITE,
                              0, // No sharing
                              NULL, // No security attributes
                              OPEN_EXISTING,
                              0, // Non-overlapped I/O. For overlapped, use FILE_FLAG_OVERLAPPED
                              NULL); // No template file

    if (hComm == INVALID_HANDLE_VALUE) {
        cpp_log_windows_error("CreateFileA", GetLastError());
        return; // Thread exits if port cannot be opened
    }
    // Assign to global only after successful open
    // This assignment is not strictly thread-safe if start/stop is called rapidly from multiple Python threads
    // but pybind11 calls are usually GIL-serialized. For robustness, a mutex could protect g_hSerial assignment.
    g_hSerial = hComm; 

    DCB dcbSerialParams = {0};
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    if (!GetCommState(g_hSerial, &dcbSerialParams)) {
        cpp_log_windows_error("GetCommState", GetLastError());
        CloseHandle(g_hSerial); g_hSerial = INVALID_HANDLE_VALUE;
        return;
    }

    dcbSerialParams.BaudRate = baud_rate;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity   = NOPARITY;
    dcbSerialParams.fDtrControl = DTR_CONTROL_DISABLE;
    dcbSerialParams.fRtsControl = RTS_CONTROL_DISABLE;
    // Other parameters if needed (fOutxCtsFlow, fOutxDsrFlow, etc.)

    if (!SetCommState(g_hSerial, &dcbSerialParams)) {
        cpp_log_windows_error("SetCommState", GetLastError());
        CloseHandle(g_hSerial); g_hSerial = INVALID_HANDLE_VALUE;
        return;
    }

    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout         = MAXDWORD; // Non-blocking for the interval part
    timeouts.ReadTotalTimeoutConstant    = SERIAL_READ_TIMEOUT_MS;
    timeouts.ReadTotalTimeoutMultiplier  = 0; // No multiplier, constant timeout
    timeouts.WriteTotalTimeoutConstant   = SERIAL_WRITE_TIMEOUT_MS;
    timeouts.WriteTotalTimeoutMultiplier = 0;

    if (!SetCommTimeouts(g_hSerial, &timeouts)) {
        cpp_log_windows_error("SetCommTimeouts", GetLastError());
        CloseHandle(g_hSerial); g_hSerial = INVALID_HANDLE_VALUE;
        return;
    }

    cpp_log_message("[C++ Backend]", "Serial port configured. Polling started.");
    g_port_initialized_successfully.store(true, std::memory_order_relaxed);

    std::vector<uint8_t> request_frame = build_modbus_read_frame(
        KINCO_SLAVE_ID, KINCO_POSITION_REGISTER_START, KINCO_REGISTERS_TO_READ_COUNT
    );
    const size_t expected_response_length = 3 + (2 * KINCO_REGISTERS_TO_READ_COUNT) + 2; // Header + Data + CRC
    std::vector<uint8_t> response_buffer(expected_response_length);

    while (g_keep_running_thread.load(std::memory_order_acquire)) {
        auto loop_start_time = std::chrono::steady_clock::now();

        DWORD bytes_written = 0;
        if (!WriteFile(g_hSerial, request_frame.data(), static_cast<DWORD>(request_frame.size()), &bytes_written, NULL)) {
            cpp_log_windows_error("WriteFile", GetLastError());
            // Consider if thread should exit or retry after some errors
            if (GetLastError() == ERROR_OPERATION_ABORTED) break; // Port likely closed by stop()
            std::this_thread::sleep_for(std::chrono::milliseconds(LOOP_SLEEP_DURATION_MS * 5)); // Longer sleep on error
            continue;
        }
        if (bytes_written != request_frame.size()) {
            g_last_error_message = "WriteFile: Incomplete write. Wrote " + 
                                   std::to_string(bytes_written) + " expected " + std::to_string(request_frame.size());
            cpp_log_message("[C++ Error]", g_last_error_message);
            // continue or break based on policy
        }

        DWORD bytes_read = 0;
        if (!ReadFile(g_hSerial, response_buffer.data(), static_cast<DWORD>(response_buffer.size()), &bytes_read, NULL)) {
            DWORD read_error = GetLastError();
            if (read_error != ERROR_TIMEOUT) { // ERROR_TIMEOUT is expected if no data in SERIAL_READ_TIMEOUT_MS
                 cpp_log_windows_error("ReadFile", read_error);
                 if (read_error == ERROR_OPERATION_ABORTED) break; // Port likely closed
            }
            // If timeout or other error, g_latest_motor_position remains unchanged
        } else {
            if (bytes_read == expected_response_length) {
                // Validate Slave ID and Function Code (basic check)
                if (response_buffer[0] == KINCO_SLAVE_ID && response_buffer[1] == 0x03) {
                    uint16_t received_crc = (static_cast<uint16_t>(response_buffer[bytes_read - 1]) << 8) | 
                                             static_cast<uint16_t>(response_buffer[bytes_read - 2]);
                    if (calculate_crc16(response_buffer.data(), bytes_read - 2) == received_crc) {
                        // Data payload is at index 3 for function code 0x03 (byte count is at index 2)
                        // Kinco position is signed 32-bit, Big Endian (MSB first from docs typical for Modbus)
                        // Register 0x3700 (Hi word), 0x3701 (Lo word)
                        // Response: SlaveID, Func, ByteCount, Reg0_Hi, Reg0_Lo, Reg1_Hi, Reg1_Lo, CRC_Lo, CRC_Hi
                        // Data starts at response_buffer[3]
                        int32_t position_value = 
                            (static_cast<int32_t>(static_cast<int8_t>(response_buffer[3])) << 24) | // MSB (Reg0_Hi) with sign extension
                            (static_cast<int32_t>(response_buffer[4]) << 16) | // Reg0_Lo
                            (static_cast<int32_t>(response_buffer[5]) << 8)  | // Reg1_Hi (for 32-bit, this should be 2nd part)
                            (static_cast<int32_t>(response_buffer[6]));       // Reg1_Lo (LSB)
                        g_latest_motor_position.store(position_value, std::memory_order_relaxed);
                    } else {
                        g_last_error_message = "Modbus CRC error in response.";
                        cpp_log_message("[C++ Error]", g_last_error_message);
                    }
                } else {
                    g_last_error_message = "Modbus response: Incorrect Slave ID or Function Code.";
                    cpp_log_message("[C++ Error]", g_last_error_message);
                }
            } else if (bytes_read > 0) {
                g_last_error_message = "Modbus response: Incomplete frame. Read " + 
                                       std::to_string(bytes_read) + " expected " + std::to_string(expected_response_length);
                cpp_log_message("[C++ Error]", g_last_error_message);
            }
            // If bytes_read is 0, it's a valid timeout, no new data.
        }

        // Ensure loop runs at desired frequency
        auto loop_end_time = std::chrono::steady_clock::now();
        auto loop_duration = std::chrono::duration_cast<std::chrono::milliseconds>(loop_end_time - loop_start_time);
        if (loop_duration.count() < LOOP_SLEEP_DURATION_MS) {
            std::this_thread::sleep_for(std::chrono::milliseconds(LOOP_SLEEP_DURATION_MS - loop_duration.count()));
        }
    }

    // Cleanup when thread exits
    if (g_hSerial != INVALID_HANDLE_VALUE) {
        CloseHandle(g_hSerial);
        g_hSerial = INVALID_HANDLE_VALUE;
    }
    g_port_initialized_successfully.store(false, std::memory_order_relaxed);
    cpp_log_message("[C++ Backend]", "Serial reader thread stopped.");
}

// --- Pybind11 Interface Functions ---

bool start_hfb(const std::string& port_name, int baud_rate = 115200) {
    if (g_reader_thread.joinable()) {
        cpp_log_message("[C++ Backend]", "Reader thread already running. Stopping first...");
        g_keep_running_thread.store(false, std::memory_order_release);
        if (g_reader_thread.get_id() != std::this_thread::get_id()) { // Check not self-joining
            g_reader_thread.join();
        }
    }
    // Ensure globals are reset for a fresh start
    if (g_hSerial != INVALID_HANDLE_VALUE) { CloseHandle(g_hSerial); g_hSerial = INVALID_HANDLE_VALUE; }
    g_port_initialized_successfully.store(false, std::memory_order_relaxed);
    g_last_com_error_code.store(0, std::memory_order_relaxed);
    g_last_error_message.clear();

    g_keep_running_thread.store(true, std::memory_order_release); // Set before creating thread
    try {
        g_reader_thread = std::thread(serial_reader_loop, port_name, baud_rate);
    } catch (const std::exception& e) {
        g_keep_running_thread.store(false, std::memory_order_relaxed);
        g_last_error_message = "Failed to create reader thread: " + std::string(e.what());
        cpp_log_message("[C++ Error]", g_last_error_message);
        return false;
    }

    // Wait for the thread to initialize the port
    // A better sync mechanism (promise/future, condition_variable) is more robust
    for (int i = 0; i < 50; ++i) { // Wait up to 500ms (50 * 10ms)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if (g_port_initialized_successfully.load(std::memory_order_acquire)) {
            cpp_log_message("[C++ Backend]", "start_hfb: Confirmed port initialized by thread.");
            return true;
        }
        if (g_last_com_error_code.load(std::memory_order_relaxed) != 0) {
            cpp_log_message("[C++ Error]", "start_hfb: Port initialization failed in thread (error code set).");
            // Ensure thread is cleaned up if it exited due to error
            if (g_reader_thread.joinable()) { g_reader_thread.join(); }
            return false;
        }
        // Simple check if thread is still alive (not foolproof, but better than nothing)
        // If not joinable, it might have exited without setting flags.
        if (!g_reader_thread.joinable() && !g_port_initialized_successfully.load(std::memory_order_acquire)){
            cpp_log_message("[C++ Error]", "start_hfb: Reader thread exited prematurely without success signal.");
            // g_last_error_message might have been set by the thread if it logged before exiting.
            return false; 
        }
    }

    cpp_log_message("[C++ Error]", "start_hfb: Timeout waiting for port initialization by thread.");
    // If timeout, assume failure and try to clean up
    g_keep_running_thread.store(false, std::memory_order_release); 
    if (g_reader_thread.joinable()) { g_reader_thread.join(); }
    return false;
}

void stop_hfb() {
    cpp_log_message("[C++ Backend]", "stop_hfb called.");
    g_keep_running_thread.store(false, std::memory_order_release);
    if (g_reader_thread.joinable()) {
        if (g_reader_thread.get_id() != std::this_thread::get_id()) {
            g_reader_thread.join();
            cpp_log_message("[C++ Backend]", "Reader thread joined.");
        } else {
             cpp_log_message("[C++ Warning]", "stop_hfb: Attempt to self-join thread (called from reader thread?).");
        }
    }
    // g_hSerial is closed by the thread itself on exit.
    // Reset flags here for clarity if start is called again without re-creating module object
    g_port_initialized_successfully.store(false, std::memory_order_relaxed);
}

int32_t get_motor_position() {
    return g_latest_motor_position.load(std::memory_order_relaxed);
}

std::pair<int, std::string> get_last_hfb_error() {
    return {static_cast<int>(g_last_com_error_code.load(std::memory_order_relaxed)), g_last_error_message};
}

bool is_hfb_running() {
    // Checks if the thread is meant to be running and if port was init'd.
    // Thread might have exited due to an error even if g_keep_running_thread is true.
    // A more robust check would be if g_reader_thread.joinable() is true and not yet joined.
    return g_keep_running_thread.load(std::memory_order_acquire) && 
           g_port_initialized_successfully.load(std::memory_order_acquire);
}

// --- Pybind11 Module Definition ---
PYBIND11_MODULE(kinco_hfb, m) {
    m.doc() = "Kinco High-Frequency Backend (HFB) for Modbus RTU position reading";

    m.def("start", &start_hfb,
          "Initializes and starts the serial reader thread.",
          py::arg("port_name"), py::arg("baud_rate") = 115200);

    m.def("stop", &stop_hfb,
          "Stops the serial reader thread and closes the port.");

    m.def("get_position", &get_motor_position,
          "Returns the latest motor position read by the backend thread.");

    m.def("get_last_error", &get_last_hfb_error,
          "Returns a pair (error_code, error_message_string) of the last serial communication error.");
    
    m.def("is_running", &is_hfb_running,
          "Checks if the backend reader thread is active and port was initialized.");
} 