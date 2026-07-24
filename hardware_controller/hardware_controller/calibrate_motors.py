import json
import board
import busio
import time
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# 1. Load the hardware configuration
def load_config(filename="micro_config.json"):
    try:
        with open(filename, 'r') as file:
            return json.load(file)
    except FileNotFoundError:
        print(f"Error: Could not find {filename}. Make sure it is in the same folder.")
        exit()

config = load_config()

# 2. Initialize the I2C bus and Boards
i2c = busio.I2C(board.SCL, board.SDA)
pca_boards = {}

print("Initializing boards...")
for board_name, address in config["boards"].items():
    pca = PCA9685(i2c, address=address)
    pca.frequency = 50 
    pca_boards[board_name] = pca

# 3. Map the servos and track calibration angles
robot_joints = {}
calibration_data = {}

for joint_name, mapping in config["servos"].items():
    board_name = mapping["board"]
    pin_number = mapping["pin"]
    target_board = pca_boards[board_name]
    
    # Initialize the servo object
    robot_joints[joint_name] = servo.Servo(
        target_board.channels[pin_number],
        min_pulse=500, 
        max_pulse=2500
    )
    # Set default calibration angle to 90
    calibration_data[joint_name] = 90.0

# 4. Set all motors to exactly 90 degrees to start
print("\nMoving all joints to 90 degrees (center position)...")
for joint_name, servo_motor in robot_joints.items():
    servo_motor.angle = calibration_data[joint_name]
    time.sleep(0.1) # Prevent power brownouts by delaying slightly

print("All motors centered!")

# 5. Interactive Calibration Loop
while True:
    print("\n" + "="*40)
    print("      QUADRUPED CALIBRATION TOOL")
    print("="*40)
    
    # Display current calibration state
    joint_names = list(robot_joints.keys())
    for i in range(0, len(joint_names), 4):
        chunk = joint_names[i:i+4]
        line = " | ".join([f"{name}: {calibration_data[name]}" for name in chunk])
        print(line)
        
    print("-" * 40)
    print("Commands:")
    print(" - Type a JOINT NAME to tweak its angle (e.g., 'FR_Hip')")
    print(" - Type 'save' to export your calibration file.")
    print(" - Type 'quit' to exit.")
    
    command = input("\nEnter command: ").strip()
    
    if command.lower() == 'quit':
        print("Exiting without saving.")
        break
        
    elif command.lower() == 'save':
        with open("calibrate.json", "w") as outfile:
            json.dump(calibration_data, outfile, indent=4)
        print("\nSUCCESS: Calibration data saved to 'calibrate.json'!")
        
    elif command in robot_joints:
        angle_input = input(f"Enter new angle for {command} (Current: {calibration_data[command]}): ").strip()
        try:
            angle = float(angle_input)
            if 0 <= angle <= 180:
                robot_joints[command].angle = angle
                calibration_data[command] = angle
                print(f">>> {command} adjusted to {angle} degrees.")
            else:
                print("Error: Angle must be between 0 and 180.")
        except ValueError:
            print("Error: Please enter a valid number.")
            
    else:
        print("Error: Unrecognized command or joint name. Check your spelling.")
