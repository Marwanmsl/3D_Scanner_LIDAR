import RPi.GPIO as GPIO
import time
import smbus2
import numpy as np
import matplotlib.pyplot as plt
import trimesh
from mpl_toolkits.mplot3d import Axes3D

PAN_PIN = 12
TILT_PIN = 11
LIDAR_LITE_ADDRESS = 0x62
ACQ_COMMAND = 0x00
DISTANCE_REGISTER = 0x8f
PAN_START_ANGLE = 0
PAN_END_ANGLE = 180
TILT_START_ANGLE = 0
TILT_END_ANGLE = 180
ANGLE_STEP = 5

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.setup(PAN_PIN, GPIO.OUT)
GPIO.setup(TILT_PIN, GPIO.OUT)

pan_servo = GPIO.PWM(PAN_PIN, 50)
tilt_servo = GPIO.PWM(TILT_PIN, 50)

pan_servo.start(0)
tilt_servo.start(0)

bus = smbus2.SMBus(1)

def set_servo_angle(servo, angle):
    duty_cycle = angle / 18 + 2
    servo.ChangeDutyCycle(duty_cycle)
    time.sleep(0.5)
    servo.ChangeDutyCycle(0)

def read_lidar():
    try:
        bus.write_byte_data(LIDAR_LITE_ADDRESS, ACQ_COMMAND, 0x04)
        time.sleep(0.02)
        distance = bus.read_word_data(LIDAR_LITE_ADDRESS, DISTANCE_REGISTER)
        distance = (distance >> 8) & 0xff | (distance & 0xff) << 8
        return distance
    except Exception as e:
        print(f"Error reading LIDAR: {e}")
        return None

def main():
    points = []

    for pan_angle in range(PAN_START_ANGLE, PAN_END_ANGLE + 1, ANGLE_STEP):
        set_servo_angle(pan_servo, pan_angle)
        for tilt_angle in range(TILT_START_ANGLE, TILT_END_ANGLE + 1, ANGLE_STEP):
            set_servo_angle(tilt_servo, tilt_angle)
            distance = read_lidar()
            if distance is not None and distance > 0:
                x = distance * np.sin(np.radians(pan_angle)) * np.cos(np.radians(tilt_angle))
                y = distance * np.sin(np.radians(pan_angle)) * np.sin(np.radians(tilt_angle))
                z = distance * np.cos(np.radians(pan_angle))
                points.append([x, y, z])
                print(f"Pan: {pan_angle}, Tilt: {tilt_angle}, Distance: {distance}")

    points = np.array(points)

#     x = points[:, 0]
#     y = points[:, 1]
#     z = points[:, 2]
# 

#     fig = plt.figure()
#     ax = fig.add_subplot(111, projection='3d')
#     ax.plot_trisurf(x, y, z, cmap='viridis', edgecolor='none')
# 

#     ax.set_xlabel('X')
#     ax.set_ylabel('Y')
#     ax.set_zlabel('Z')
    
    point_cloud = trimesh.points.PointCloud(points)
    mesh = point_cloud.convex_hull
    mesh.export('lidar_scan.obj')
    print('3D model saved as lidar_scan.obj')
#     plt.show()

if __name__ == "__main__":
    try:
        main()
    finally:
        pan_servo.stop()
        tilt_servo.stop()
        GPIO.cleanup()