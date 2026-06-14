from mpu6050 import mpu6050


class ImuDriver:
    def __init__(self, address=0x68, calibration=None):
        self.sensor = mpu6050(address)

        calibration = calibration or {}

        self.gyro_offset = calibration.get(
            "gyro",
            {"x": 0.0, "y": 0.0, "z": 0.0}
        )

        self.accel_offset = calibration.get(
            "accel",
            {"x": 0.0, "y": 0.0, "z": 0.0}
        )

    def read(self):
        accel = self.sensor.get_accel_data()
        gyro = self.sensor.get_gyro_data()

        corrected_accel = {
            "x": accel["x"] - self.accel_offset["x"],
            "y": accel["y"] - self.accel_offset["y"],
            "z": accel["z"] - self.accel_offset["z"] + 9.81,
        }

        corrected_gyro = {
            "x": gyro["x"] - self.gyro_offset["x"],
            "y": gyro["y"] - self.gyro_offset["y"],
            "z": gyro["z"] - self.gyro_offset["z"],
        }

        return {'accel' : corrected_accel, 'gyro' : corrected_gyro}
