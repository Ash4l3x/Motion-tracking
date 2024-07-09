import os
import matplotlib.pyplot as plt
import plotly.graph_objects as go
import time
import struct
import serial

def sort_foldernames(folder_names:list)->list:
    return sorted(folder_names, key=lambda x: int(x.split('_')[1]))

def get_folder_name():
    current_dir=os.path.dirname(os.path.realpath(__file__))
    os.chdir(current_dir)
    if not os.path.exists("./output"):
        os.mkdir("./output")
    all_runs = sort_foldernames(os.listdir("./output"))
    last_run=0
    if len(all_runs) == 0:
        last_run=1
    else:
        last_run=all_runs[-1]
        last_run=last_run.replace("run_", " ")
        last_run=int(last_run)
        last_run = last_run + 1
    new_folder=f"./output/run_{last_run}"
    os.mkdir(new_folder)
    return new_folder

FOLDER_PATH = get_folder_name()
FILE_NAME = "raw_data.txt"

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

class Matrix:
    def __init__(self, columns:int, rows:int, initial_value=0):
        self.columns=columns
        self.rows=rows
        self.matrix_list= []
        for i in range(self.rows):
            row = MatrixRow(self.columns, initial_value)
            self.matrix_list.append(row)

    def __str__(self):
        for i in range(len(self.matrix_list)):
            print(self.matrix_list[i])
        return ""
    
    def __getitem__(self, key):
        return self.matrix_list[key]

    def __setitem__(self, key, value):
        self.matrix_list[key] = value
    
    def get_row(self, row_index:int)->list:
        return self.matrix_list[row_index]
    
    def get_column(self, column_index:int)->list:
        aux_list=[]
        for i in range(len(self.matrix_list)):
            aux_list.append(self.matrix_list[i][column_index])
        return aux_list

class MatrixRow:
    def __init__(self, columns, value):
        self.columns=columns
        self.row = [MatrixElement(value) for _ in range(columns)]

    def __str__(self):
        return str(self.row)
    
    def __getitem__(self, key):
        return self.row[key]

    def __setitem__(self, key, value):
        self.row[key] = value

    def __len__(self):
        return len(self.row)
    
    def __delitem__(self, key):
        del self.row[key]

    def __class__(self, key):
        return type(key)

class MatrixElement:
    def __init__(self, value):
        self.value = value * 1

    def __str__(self):
        return str(self.value)
    
    def __repr__(self) -> str:
        return str(self.value)
    
    def __getitem__(self, key):
        return self.value[key]

    def __setitem__(self, key, value):
        self.value[key] = value

    def __eq__(self, other):
        if isinstance(other, MatrixElement):
            return self.value == other.value
        else:
            return self.value == other

class Data_structure:
    def __init__(self, sensor_name):
        os.mkdir(f"{FOLDER_PATH}/{sensor_name}")
        self.file_path=f"{FOLDER_PATH}/{sensor_name}/{FILE_NAME}"
        if os.path.exists(self.file_path):
            os.remove(self.file_path)
        self.file_obj = open(self.file_path, "w")
        self.time_start=0

    def insert_data(self, a, b, c, d, e, f, time):
        self.time_start+=time
        self.file_obj.write(f"{self.time_start} : {a} {b} {c} {d} {e} {f}\n")
    
    def save_structure(self):
        self.file_obj.close()

class Orientation:
    vel_x_accel=0.0
    vel_y_accel=0.0
    vel_z_accel=0.0
    vel_x_rot=0.0
    vel_y_rot=0.0
    vel_z_rot=0.0

    def __init__(self, sensor_name):
        self.data_struct=Data_structure(sensor_name)
        self.x_pos=0.0
        self.y_pos=0.0
        self.z_pos=0.0
        self.x_rot=0.0
        self.y_rot=0.0
        self.z_rot=0.0
        time=0.0
        
    def init_global_world_values(self, x_accel, y_accel, z_accel, x_rot, y_rot, z_rot):
        self.x_pos=x_accel
        self.y_pos=y_accel
        self.z_pos=z_accel
        self.x_rot=x_rot
        self.y_rot=y_rot
        self.z_rot=z_rot

    def integrate_acceleration_accel(self, x, y, z, time):
        self.vel_x_accel = self.vel_x_accel + (x * time)
        self.vel_y_accel = self.vel_y_accel + (y * time)
        self.vel_z_accel = self.vel_z_accel + (z * time)

    def integrate_velocity_accel(self, time):
        self.x_pos = self.x_pos + (self.vel_x_accel * time)
        self.y_pos = self.y_pos + (self.vel_y_accel * time)
        self.z_pos = self.z_pos + (self.vel_z_accel * time)

    def integrate_acceleration_rot(self, x, y, z, time):
        self.vel_x_rot = self.vel_x_rot + (x * time)
        self.vel_y_rot = self.vel_y_rot + (y * time)
        self.vel_z_rot = self.vel_z_rot + (z * time)

    def integrate_velocity_rot(self, time):
        self.x_rot = self.x_rot + (self.vel_x_rot * time)
        self.y_rot = self.y_rot + (self.vel_y_rot * time)
        self.z_rot = self.z_rot + (self.vel_z_rot * time)

    def normalise_values(self):
        self.x_pos=round(self.x_pos, 2)
        self.y_pos=round(self.y_pos, 2)
        self.z_pos=round(self.z_pos, 2)
        
        if self.x_rot >= 360:
            self.x_rot = self.x_rot - 360
        if self.y_rot >= 360:
            self.y_rot = self.y_rot - 360
        if self.z_rot >= 360:
            self.z_rot = self.z_rot - 360

        self.x_rot=round(self.x_rot, 2)
        self.y_rot=round(self.y_rot, 2)
        self.z_rot=round(self.z_rot, 2)

    def convert_time(self, time):
        self.time=round(time/100, 4)
    
    def update_global_position(self, new_xa, new_ya, new_za, new_xr, new_yr, new_zr, time):
        self.convert_time(time)
        self.integrate_acceleration_accel(new_xa, new_ya, new_za, self.time)
        self.integrate_acceleration_rot(new_xr, new_zr, new_yr, self.time)
        self.integrate_velocity_accel(self.time)
        self.integrate_velocity_rot(self.time)
        self.normalise_values()

        self.data_struct.insert_data(self.x_pos, self.y_pos, self.z_pos, self.x_rot, self.y_rot, self.z_rot, time)

    def stop(self):
        self.data_struct.save_structure()

class Data_interpretation:
    file_contents=""
    file_lines=[]
    matrix=None
    matrix_width=7
    matrix_height=None
    folder_2d=""
    folder_3d=""
    x_accel_column=[]
    y_accel_column=[]
    z_accel_column=[]
    x_rot_column=[]
    y_rot_column=[]
    z_rot_column=[]
    time_column=[]
    colours_list=[]

    def __init__(self, sensor_name):
        path=f"{FOLDER_PATH}/{sensor_name}/{FILE_NAME}"
        with open(path, "r") as file:
            self.file_contents=file.read()
        file.close()
        self.folder_2d=f"{FOLDER_PATH}/{sensor_name}/plots_2d"
        self.folder_3d=f"{FOLDER_PATH}/{sensor_name}/plots_3d"
        self.extract_data()
        os.mkdir(self.folder_2d)
        os.mkdir(self.folder_3d)

    def extract_data(self):
        self.file_lines=self.file_contents.split("\n")
        self.matrix_height=len(self.file_lines)-1
        self.init_matrix()
        for i in range(self.matrix_height):
            time_data = self.file_lines[i].split(":")
            self.matrix[i][0]=int(time_data[0].strip())
            data_split=time_data[1].split(" ")
            for j in range(1, self.matrix_width):
                self.matrix[i][j]=float(data_split[j])
        self.init_colour_list()
        self.extract_plotting_columns()

    def init_matrix(self):
        self.matrix=Matrix(self.matrix_width, self.matrix_height, 0)

    def extract_plotting_columns(self):
        self.x_accel_column=self.matrix.get_column(1)
        self.y_accel_column=self.matrix.get_column(2)
        self.z_accel_column=self.matrix.get_column(3)
        self.x_rot_column=self.matrix.get_column(4)
        self.y_rot_column=self.matrix.get_column(3)
        self.z_rot_column=self.matrix.get_column(5)
        self.time_column=self.matrix.get_column(0)
    
    def plot_2d(self):
        #####################################################
        plt.plot(self.time_column, self.x_accel_column)
        plt.title("Position X")
        plt.ylabel("X")
        plt.xlabel("TIME")
        plt.savefig(f"{self.folder_2d}/X_Movement.png")
        plt.close()
        #####################################################
        plt.plot(self.time_column, self.y_accel_column)
        plt.title("Position Y")
        plt.ylabel("Y")
        plt.xlabel("TIME")
        plt.savefig(f"{self.folder_2d}/Y_Movement.png")
        plt.close()
        #####################################################
        plt.plot(self.time_column, self.z_accel_column)
        plt.title("Position Z")
        plt.ylabel("Z")
        plt.xlabel("TIME")
        plt.savefig(f"{self.folder_2d}/Z_Movement.png")
        plt.close()
        #####################################################
        plt.plot(self.time_column, self.x_rot_column)
        plt.title("ROTATION X")
        plt.ylabel("X")
        plt.xlabel("TIME")
        plt.savefig(f"{self.folder_2d}/X_Rotation.png")
        plt.close()
        #####################################################
        plt.plot(self.time_column, self.y_rot_column)
        plt.title("ROTATION Y")
        plt.ylabel("Y")
        plt.xlabel("TIME")
        plt.savefig(f"{self.folder_2d}/Y_Rotation.png")
        plt.close()
        #####################################################
        plt.plot(self.time_column, self.z_rot_column)
        plt.title("ROTATION Z")
        plt.ylabel("Z")
        plt.xlabel("TIME")
        plt.savefig(f"{self.folder_2d}/Z_Rotation.png")
        plt.close()

    def plot_3d(self):
        fig = go.Figure(data=[go.Scatter3d(
        x=self.x_accel_column,
        y=self.y_accel_column,
        z=self.z_accel_column,
        mode='lines+markers',
        marker=dict(
            size=5,
            color=self.colours_list,
            opacity=1
        )
        )])

        fig.update_layout(scene=dict(
            xaxis_title='X AXIS',
            yaxis_title='Y AXIS',
            zaxis_title='Z AXIS'),
            width=1500,
            margin=dict(r=20, l=10, b=10, t=10)
        )
        fig.write_html(f"{self.folder_3d}/3d_scatter_plot_position.html")

        ######################################################################

        fig = go.Figure(data=[go.Scatter3d(
        x=self.x_rot_column,
        y=self.y_rot_column,
        z=self.z_rot_column,
        mode='lines+markers',
        marker=dict(
            size=5,
            color=self.colours_list,
            opacity=1
        )
        )])

        fig.update_layout(scene=dict(
            xaxis_title='X AXIS ROT',
            yaxis_title='Y AXIS ROT',
            zaxis_title='Z AXIS ROT'),
            width=1500,
            margin=dict(r=20, l=10, b=10, t=10)
        )
        fig.write_html(f"{self.folder_3d}/3d_scatter_plot_rotation.html")
    
    def init_colour_list(self):
        for i in range(self.matrix_height):
            if i<5:
                self.colours_list.append("red")
            else:
                self.colours_list.append("blue")


if __name__=="__main__":
    sensor1="left_hand"
    sensor2="right_hand"
    sensor3="external"
    coms=START_COM("COM1", 460800)
    time.sleep(5)

    position1=Orientation(sensor1)
    position2=Orientation(sensor2)
    position3=Orientation(sensor3)

    mov_data = ()

    for i in range(0, 100):
        print(i)
        coms.send("2")
        data=coms._read(72)
        mov_data = struct.unpack('18f', data)
        print(mov_data)
        position1.update_global_position(mov_data[0], mov_data[1], mov_data[2], mov_data[3], mov_data[4], mov_data[5], 100)
        position2.update_global_position(mov_data[6], mov_data[7], mov_data[8], mov_data[9], mov_data[10], mov_data[11], 100)
        position3.update_global_position(mov_data[12], mov_data[13], mov_data[14], mov_data[15], mov_data[16], mov_data[17], 100)
        time.sleep(0.1)

    coms.close_coms()

    position1.stop()
    position2.stop()
    position3.stop()

    data_interp1=Data_interpretation(sensor1)
    data_interp1.plot_2d()
    data_interp1.plot_3d()

    data_interp2=Data_interpretation(sensor2)
    data_interp2.plot_2d()
    data_interp2.plot_3d()

    data_interp3=Data_interpretation(sensor3)
    data_interp3.plot_2d()
    data_interp3.plot_3d()
