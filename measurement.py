import smbus
import math
from time import sleep
from smbus import SMBus
import RPi.GPIO as GPIO
import signal
import sys
import csv

# slaveaddress
DEV_ADDR = 0x68         # device address
# register address
ACCEL_XOUT = 0x3b
ACCEL_YOUT = 0x3d
ACCEL_ZOUT = 0x3f
TEMP_OUT = 0x41
GYRO_XOUT = 0x43
GYRO_YOUT = 0x45
GYRO_ZOUT = 0x47
PWR_MGMT_1 = 0x6b       # PWR_MGMT_1
PWR_MGMT_2 = 0x6c       # PWR_MGMT_2

bus = smbus.SMBus(1)
bus.write_byte_data(DEV_ADDR, PWR_MGMT_1, 0)

def read_byte(adr):
    return bus.read_byte_data(DEV_ADDR, adr)

def read_word(adr):
    high = bus.read_byte_data(DEV_ADDR, adr)
    low = bus.read_byte_data(DEV_ADDR, adr+1)
    val = (high << 8) + low
    return val

def read_word_sensor(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def get_gyro_data_lsb():               #角速度(ジャイロ)データ取得
    x = read_word_sensor(GYRO_XOUT)
    y = read_word_sensor(GYRO_YOUT)
    z = read_word_sensor(GYRO_ZOUT)
    return [x, y, z]

def get_gyro_data_deg():
    x,y,z = get_gyro_data_lsb()
    x = x / 131.0
    y = y / 131.0
    z = z / 131.0
    return [x, y, z]

def get_accel_data_lsb():              #加速度データ取得
    x = read_word_sensor(ACCEL_XOUT)
    y = read_word_sensor(ACCEL_YOUT)
    z = read_word_sensor(ACCEL_ZOUT)
    return [x, y, z]

def get_accel_data_g():
    x,y,z = get_accel_data_lsb()
    x = ((x / 16384.0) *1)
    y = ((y / 16384.0) *1)
    z = ((z / 16384.0) *1)
    return [x, y, z]

calculate_time= 30
dt            = 0.1
time          = 0

while 1:
     accel_x,accel_y,accel_z = get_accel_data_g()
     gyro_x, gyro_y, gyro_z = get_gyro_data_deg()

     y_angle = math.degrees(math.atan2(accel_x , math.sqrt(accel_y**2 + accel_z**2)))          #姿勢角の算出
     y_angle = (0.995 * (y_angle + gyro_y * dt) + (0.005* accel_y))
     #print(round(y_angle,0) ,"度")
     y_angle = math.radians(y_angle)

     with open('measurement.csv','a', newline='') as measurement_file:
        writer = csv.writer(measurement_file,lineterminator='\n')
        writer.writerow([accel_x,accel_y,accel_z,y_angle])

     time += dt
     if time > calculate_time:
        break

     sleep(dt)
