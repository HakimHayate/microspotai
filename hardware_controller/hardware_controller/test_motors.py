import json
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

def load_config(filename="micro_config.json"):
    with open(filename, 'r') as file:
        return json.load(file)

config = load_config()

i2c = busio.I2C(board.SCL, board.SDA)

pca_boards = {}
for board_name, address in config["boards"].items():
    pca = PCA9685(i2c, address=address)
    pca.frequency = 50 
    pca_boards[board_name] = pca

robot_joints = {}
for joint_name, mapping in config["servos"].items():
    board_name = mapping["board"]
    pin_number = mapping["pin"]
    target_board = pca_boards[board_name]
    
    robot_joints[joint_name] = servo.Servo(
        target_board.channels[pin_number],
        min_pulse=500, 
        max_pulse=2500
    )

print("\n--- Quadruped Motor Tester ---")
print("Boards initialized successfully.")

while True:
    print("\nAvailable Joints:")
    joint_names = list(robot_joints.keys())
    for i in range(0, len(joint_names), 4):
        print(f"  {', '.join(joint_names[i:i+4])}")
        
    print("\nType 'quit' to exit.")
    
    target_joint = input("\nEnter the name of the joint you want to move (e.g., FR_Hip): ").strip()
    
    if target_joint.lower() == 'quit':
        print("Exiting tester. Goodbye!")
        break
        
    if target_joint not in robot_joints:
        print(f"Error: '{target_joint}' is not a valid joint name. Check your spelling.")
        continue
        
    angle_input = input(f"Enter the target angle for {target_joint} (0 to 180): ").strip()
    
    try:
        angle = float(angle_input)
        if angle < 0 or angle > 180:
            print("Error: Angle must be between 0 and 180 degrees.")
            continue
            
        print(f">>> Moving {target_joint} to {angle} degrees...")
        robot_joints[target_joint].angle = angle
        print("Done.")
        
    except ValueError:
        print("Error: Please enter a valid number for the angle.")
