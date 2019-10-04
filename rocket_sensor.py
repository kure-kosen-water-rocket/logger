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

def get_temp():                        #温度データ取得
    temp = read_word_sensor(TEMP_OUT)
    x = temp / 340 + 36.53 
    return x

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
    x = ((x / 16384.0) *10) + 0.2      # *10=m/sに変換,  +0.x=加速度調整パラメーター
    y = ((y / 16384.0) *10) -0.07
    z = ((z / 16384.0) *10) +0.3
    return [x, y, z]

#----------------------------------------------------------------------------------------------------------------------------------------------------------------
dt            = 0.1
time          = 0
filterCoefficient = 0.5  #ローパスフィルターの係数(これは環境によって要調整。1に近づけるほど平滑化の度合いが大きくなる。

x_lowpassValue  = 0
x_highpassValue = 0
x_speed       = 0        #加速度時から算出した速度
x_oldSpeed    = 0        #ひとつ前の速度
x_oldAccel    = 0        #ひとつ前の加速度

z_lowpassValue  = 0
z_highpassValue = 0
z_speed       = 0
z_oldSpeed    = 0
z_oldAccel    = 0

sensor = Bme280()        #BME280インスタンス化

with open('fly_distance.csv', 'W', newline='') as fly_distance_file:                          #fly_distance.csvファイルを作成
    fieldnames = ['x_difference','z_difference']                                                      #一列目に'time'、二列目に'displacement'
    write = csv.Dictwriter(fly_distance_file, fieldnames=fieldnames)                          #temperature_file(temperature.csv)にfieldnamesの設定を反映
    writer.writeheader()

while 1:
    gyro_x,gyro_y,gyro_z = get_gyro_data_deg()                                                #角速度表示
    #print ('gyro[deg/s]')
    #print ('x: %08.3f' % gyro_x)
    #print ('y: %08.3f' % gyro_y)
    #print ('z: %08.3f' % gyro_z)
    #print ('||')
    
    accel_x,accel_y,accel_z = get_accel_data_g()                                              #加速度表示
    #print ('accel[g]')
    #print ('x: %06.3f' % accel_x)
    #print ('y: %06.3f' % accel_y)
    #print ('z: %06.3f' % accel_z)
    
    y_angle = math.degrees(math.atan2(accel_x , math.sqrt(accel_y**2 + accel_z**2)))          #姿勢角の算出
    y_angle = 0.995 * (y_angle + gyro_y * dt) + (0.005* accel_y)
    #print(round(y_angle,0) ,"度")
    #y_angle = math.radians(y_angle)
    
    x_difference    = 0                                                                                                 #変位を初期化　　※ロケット発射時は取り除いておく
    x_lowpassValue  = (x_lowpassValue * filterCoefficient) + (accel_x) * (1 - filterCoefficient)
    x_highpassValue = (accel_x) - x_lowpassValue
    #print(x_highpassValue)
    x_speed         = ((x_highpassValue + (x_oldAccel)) * dt) / 2 + (x_speed)                                           #速度計算(加速度を台形積分する)
    x_oldAccel      = x_highpassValue
    x_difference    = abs(((((x_speed * math.cos(y_angle)) + (x_oldSpeed * math.cos(y_angle))) * dt) / 2 + difference)) #変位計算(速度を台形積分する)
    x_oldSpeed      = x_speed
    #print(x_difference*1000) 　　　　　　　　　　　　　　　　　　　　　　  #変位をmmに変換して表示

    z_difference    = 0                                                                                                 #変位を初期化　　※ロケット発射時は取り除いておく
    z_lowpassValue  = (x_lowpassValue * filterCoefficient) + (accel_z) * (1 - filterCoefficient)
    z_highpassValue = (accel_z) - z_lowpassValue
    #print(z_highpassValue)
    z_speed         = ((z_highpassValue + (z_oldAccel)) * dt) / 2 + (z_speed)                                           #速度計算(加速度を台形積分する)
    z_oldAccel      = z_highpassValue
    z_difference    = abs(((((z_speed * math.cos(y_angle)) + (z_oldSpeed * math.cos(y_angle))) * dt) / 2 + difference)) #変位計算(速度を台形積分する)
    z_oldSpeed      = z_speed
    #print(z_difference*1000) 　　　　　　　　　　　　　　　　　　　　　　  #変位をmmに変換して表示
    
    print("####################################################")
    time += dt                                                          #時間を+0.1して値を返す
    writer.writerow({'x_difference': 'x_difference', 'z_difference': 'z_difference'})

    if time > 30.0:                                                     #30s
        break
    
    sleep(dt)                                                           #0.1秒周期で繰り返す