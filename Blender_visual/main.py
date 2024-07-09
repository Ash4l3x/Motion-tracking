import bpy
import math
import time
import struct
import serial
import numpy as np
from scipy.spatial.transform import Rotation as R


rotation_bone_lower_name = "L.lower_arm"
rotation_bone_upper_name = "L.upper_arm"


if bpy.context.object.mode != 'OBJECT':
    bpy.ops.object.mode_set(mode='OBJECT')

armature_name = "Armature"
armature = bpy.data.objects.get(armature_name)

bpy.context.view_layer.objects.active = armature

bpy.ops.object.mode_set(mode='POSE')
bpy.ops.pose.select_all(action='DESELECT')

rotation_bone_lower = armature.pose.bones[rotation_bone_lower_name]
rotation_bone_upper = armature.pose.bones[rotation_bone_upper_name]

rotation_bone_lower.bone.select = True
rotation_bone_upper.bone.select = True

armature.data.bones.active = rotation_bone_lower.bone
armature.data.bones.active = rotation_bone_upper.bone

rotation_bone_lower.rotation_mode = 'XYZ'  
rotation_bone_upper.rotation_mode = 'XYZ' 

shield = bpy.context.selectable_objects[1] 

class START_COM:
    def __init__(self, arduino_port: str, baud_rate: int):
        self.arduino_port = arduino_port
        self.baud_rate = baud_rate
        self.serial_conn = None
        self.open_coms()

    def open_coms(self):
        print(self.arduino_port, self.baud_rate)
        self.serial_conn = serial.Serial(self.arduino_port, self.baud_rate, timeout=10)

    def _read(self, size) -> bytes:
        try:
            if self.serial_conn and self.serial_conn.is_open:
                data = self.serial_conn.read(size)
                return data
            else:
                print("Serial connection not open. Cannot read data.")
        except Exception as e:
            print(f"ERROR: Failed to read data: \n{e}")

    def send(self, message) -> None:
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.write(message.encode())
        else:
            print("Serial connection not open. Cannot send message.")

    def close_coms(self) -> None:
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()

class Orientation:
    def __init__(self, sensor_name):
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.orientation = np.zeros(3)
        self.name=sensor_name

    def update_global_position(self, new_xa, new_ya, new_za, new_xr, new_yr, new_zr, time):
        self.orientation += np.array([new_xr, new_yr, new_zr]) * time
        self.orientation = np.mod(self.orientation, 360)

    def get_rotation_matrix(self):
        roll, pitch, yaw = np.radians(self.orientation)

        R_x = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])

        R_y = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])

        R_z = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])

        R = np.dot(R_z, np.dot(R_y, R_x))
        return R

    def get_euler_angles(self):
        return self.orientation
    

position1 = Orientation("left_arm_lower")
position2 = Orientation("left_arm_upper")
position3 = Orientation("external")

original_scale_x = 0
original_scale_y = -5
original_scale_z = 3

try:
    coms = START_COM("COM1", 460800)
    time.sleep(3)

    for i in range(1000):
        print(i)
        start_time = time.time()
        coms.send("2")
        data = coms._read(72)
        end_time = time.time()-start_time
        print(f"Comm duration : {end_time}\n")
        if data and len(data) == 72:
            mov_data = struct.unpack('18f', data)
            print(mov_data)
            position1.update_global_position(mov_data[0], mov_data[1], mov_data[2], mov_data[3], mov_data[4], mov_data[5], end_time)
            position2.update_global_position(mov_data[6], mov_data[7], mov_data[8], mov_data[9], mov_data[10], mov_data[11], end_time)
            position3.update_global_position(mov_data[12], mov_data[13], mov_data[14], mov_data[15], mov_data[16], mov_data[17], end_time)
            
            position_lower = position1.position
            rotation_lower = position1.get_euler_angles()
            new_rotation_radians_lower = [math.radians(angle) for angle in rotation_lower]
            rotation_bone_lower.rotation_euler = tuple(new_rotation_radians_lower)

            position_upper = position2.position
            rotation_upper = position2.get_euler_angles()
            new_rotation_radians_upper = [math.radians(angle) for angle in rotation_upper]
            rotation_bone_upper.rotation_euler = tuple(new_rotation_radians_upper)
            
            rotation_shield = position3.get_euler_angles()
            new_rotation_radians_shield = [math.radians(angle) for angle in rotation_shield]
            shield.rotation_euler = tuple(new_rotation_radians_shield)

            location_armend = armature.matrix_world @ rotation_bone_lower.tail

            shield.location.x = location_armend.x
            shield.location.y = location_armend.y - 1
            shield.location.z = location_armend.z
            
            bpy.ops.wm.redraw_timer(type="DRAW_WIN_SWAP", iterations=1)


    coms.close_coms()
except Exception as e:
    print(f"An error occurred: {e}")
    coms.close_coms()
