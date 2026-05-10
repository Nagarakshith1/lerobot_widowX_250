import time
from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus


def interactive_motor_calibration(bus: FeetechMotorsBus):
    """
    Iterates through all motors on the bus, prompting the user to 
    apply configurations, skip, or quit.
    """
    print("\n--- Starting Interactive Motor Calibration ---")

    # 1. Disable Torque globally before doing hardware resets
    bus.disable_torque()
    print("Torque disabled for all motors.\n")

    # Iterate over all motors defined in the bus
    for motor_name in bus.motors.keys():
        print(f"--- Motor: {motor_name.upper()} ---")

        while True:
            user_input = input(
                f"Actions for '{motor_name}':\n"
                f"  [c] Confirm and apply midpoint reset\n"
                f"  [s] Skip this motor\n"
                f"  [q] Quit calibration entirely\n"
                f"> "
            ).strip().lower()

            if user_input == 'q':
                print("Exiting calibration early.")
                return  # Break out of the function entirely

            elif user_input == 's':
                print(f"Skipping {motor_name}...\n")
                break  # Break the while loop, move to the next motor

            elif user_input == 'c':
                try:
                    # 2. Force the Midpoint Reset
                    bus.write("Torque_Enable", motor_name, 128, normalize=False)
                    print(f"Successfully wrote to {motor_name}.")

                    # Read back the position to verify
                    time.sleep(0.1)  # Small buffer for the write to register
                    raw_pos = bus.read("Present_Position", motor_name, normalize=False)
                    print(f"Current Raw Position for {motor_name}: {raw_pos}\n")

                except Exception as e:
                    print(f"Error communicating with {motor_name}: {e}\n")

                break

            else:
                print("Invalid input. Please press 'c', 's', or 'q'.\n")

    print("--- Calibration Complete ---")


def main():
    norm_mode_body = MotorNormMode.RANGE_M100_100

    bus = FeetechMotorsBus(
        port='/dev/ttyACM0',
        motors={
            "shoulder_pan": Motor(1, "sts3215", norm_mode_body),
            "shoulder_lift": Motor(2, "sts3215", norm_mode_body),
            "elbow_flex": Motor(3, "sts3215", norm_mode_body),
            "wrist_flex": Motor(4, "sts3215", norm_mode_body),
            "wrist_roll": Motor(5, "sts3215", norm_mode_body),
            "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
        }
    )

    try:
        bus.connect()
        interactive_motor_calibration(bus)

    except Exception as e:
        print(f"Bus connection or execution failed: {e}")

    finally:
        bus.disconnect()
        print("Bus disconnected.")


if __name__ == "__main__":
    main()