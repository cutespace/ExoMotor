# highfrequencycode/main_app_example.py
# Author: Gemini AI
# Description: Example Python application using the Kinco HFB C++ backend.

import time
import argparse
import sys

try:
    import kinco_hfb # Try to import the C++ backend module
except ImportError:
    print("Error: Could not import the 'kinco_hfb' module.")
    print("Please ensure that the C++ backend has been compiled successfully using setup.py,")
    print("and the resulting .pyd (Windows) or .so (Linux) file is in the same directory or Python path.")
    print("Compilation command: python setup.py build_ext --inplace")
    sys.exit(1)

def run_motor_test(port: str, baud: int, duration_seconds: int, poll_interval_ms: int):
    """
    Tests the Kinco HFB module by starting it, reading positions periodically,
    and then stopping it.
    """
    print(f"--- Kinco HFB Test Initializing ---")
    print(f"Attempting to start backend on port: {port}, baud: {baud}")

    if not kinco_hfb.start(port, baud):
        print("Failed to start the Kinco HFB backend.")
        error_code, error_msg = kinco_hfb.get_last_error()
        if error_code != 0 or error_msg:
            print(f"  Error Code: {error_code}")
            print(f"  Error Message: {error_msg}")
        print("Exiting test.")
        return

    print("Kinco HFB backend started successfully.")
    print(f"Will poll for position data every {poll_interval_ms} ms for {duration_seconds} seconds.")
    print("Press Ctrl+C to stop earlier.")

    start_time = time.time()
    end_time = start_time + duration_seconds
    read_count = 0
    error_count = 0

    try:
        while time.time() < end_time:
            if not kinco_hfb.is_running():
                print("Error: Backend is no longer running!")
                error_code, error_msg = kinco_hfb.get_last_error()
                if error_code != 0 or error_msg:
                    print(f"  Last Error Code: {error_code}")
                    print(f"  Last Error Message: {error_msg}")
                break

            position = kinco_hfb.get_position()
            read_count += 1
            current_time_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
            print(f"[{current_time_str}] Position: {position} (Read #{read_count})")
            
            # Check for errors periodically if needed, though get_position itself is just a read
            # error_code, _ = kinco_hfb.get_last_error() 
            # if error_code != 0:
            #    error_count +=1
            #    print(f"Warning: A C++ side error was reported (code: {error_code}) but continuing.")
            #    # Reset error or handle as needed

            time.sleep(poll_interval_ms / 1000.0)

    except KeyboardInterrupt:
        print("\nTest interrupted by user.")
    except Exception as e:
        print(f"\nAn unexpected error occurred during polling: {e}")
    finally:
        print("--- Kinco HFB Test Finalizing ---")
        print("Stopping the Kinco HFB backend...")
        kinco_hfb.stop()
        print("Kinco HFB backend stopped.")
        print(f"Total reads: {read_count}")
        if error_count > 0:
            print(f"Number of error occurrences reported by C++ side: {error_count}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Test the Kinco High-Frequency Backend (HFB).")
    parser.add_argument("--port", type=str, required=True,
                        help="Serial port name (e.g., COM11 on Windows, /dev/ttyUSB0 on Linux).")
    parser.add_argument("--baud", type=int, default=115200,
                        help="Baud rate for serial communication (default: 115200).")
    parser.add_argument("--duration", type=int, default=10,
                        help="Duration of the test in seconds (default: 10).")
    parser.add_argument("--interval", type=int, default=100,
                        help="Polling interval for get_position() in milliseconds (default: 100ms).")

    args = parser.parse_args()

    run_motor_test(args.port, args.baud, args.duration, args.interval)
    print("Test finished.") 