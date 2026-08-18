from pathlib import Path
import json
import time

from mpu6050 import mpu6050


I2C_ADDRESS = 0x68
NUM_SAMPLES = 10000
SAMPLE_DELAY = 0.005

CALIBRATION_FILE = Path("imu_calibration.json")


def collect_samples(sensor: mpu6050, num_samples: int) -> tuple[dict, dict]:
    gyro_sum = {"x": 0.0, "y": 0.0, "z": 0.0}
    accel_sum = {"x": 0.0, "y": 0.0, "z": 0.0}

    for _ in range(num_samples):
        gyro = sensor.get_gyro_data()
        accel = sensor.get_accel_data()

        for axis in ("x", "y", "z"):
            gyro_sum[axis] += gyro[axis]
            accel_sum[axis] += accel[axis]

        time.sleep(SAMPLE_DELAY)

    gyro_avg = {
        axis: gyro_sum[axis] / num_samples
        for axis in ("x", "y", "z")
    }

    accel_avg = {
        axis: accel_sum[axis] / num_samples
        for axis in ("x", "y", "z")
    }

    return gyro_avg, accel_avg


def save_calibration(filepath: Path, gyro: dict, accel: dict) -> None:
    calibration = {
        "gyro": gyro,
        "accel": accel,
    }

    with filepath.open("w") as f:
        json.dump(calibration, f, indent=4)


def load_calibration(filepath: str) -> dict:
    filepath = Path(filepath)
    
    if not filepath.exists():
        return {}

    with filepath.open('r') as f:
        return json.load(f)
  

def main() -> None:
    sensor = mpu6050(I2C_ADDRESS)

    print("Keep the IMU perfectly still...")
    time.sleep(3)

    gyro_offsets, accel_offsets = collect_samples(
        sensor=sensor,
        num_samples=NUM_SAMPLES,
    )

    save_calibration(
        filepath=CALIBRATION_FILE,
        gyro=gyro_offsets,
        accel=accel_offsets,
    )

    print("\nCalibration saved:")
    print(json.dumps({
        "gyro": gyro_offsets,
        "accel": accel_offsets,
    }, indent=4))


if __name__ == "__main__":
    main()
