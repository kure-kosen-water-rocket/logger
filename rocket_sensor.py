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
    x = ((x / 16384.0) *10)            # *10 => m/sに変換
    y = ((y / 16384.0) *10)
    z = ((z / 16384.0) *10)
    return [x, y, z]

#----------------------------------------------------------------------------------------------------------------------------------------------------------------
calculate_time= 30          #計測時間
dt            = 0.1
time          = 0           #初期タイム
x_filterCoefficient = 0.01  #ローパスフィルターの係数(これは環境によって要調整。1に近づけるほど平滑化の度合いが大きくなる。
x_lowpassValue  = 0         #加速度のみの値
x_highpassValue = 0         #加速度を取り除いた値
x_speed       = 0           #加速度時から算出した速度
x_oldSpeed    = 0           #ひとつ前の速度
x_oldAccel    = 0           #ひとつ前の加速
x_difference  = 0

z_filterCoefficient = 0.01
z_lowpassValue  = 0
z_highpassValue = 0
z_speed       = 0
z_oldSpeed    = 0
z_oldAccel    = 0
z_difference  = 0

with open('fly_distance.csv','w', newline='') as fly_distance_file:                            #'fly_distance.csv'というファイルを作成する
        fieldnames = ['x_difference','z_difference']                                
        writer = csv.DictWriter(fly_distance_file, fieldnames=fieldnames)
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
    y_angle = (0.995 * (y_angle + gyro_y * dt) + (0.005* accel_y))
    #print(round(y_angle,0) ,"度")
    y_angle = math.radians(y_angle)
    
    x_difference    = 0                                                                                                 #変位を初期化　　※ロケット発射時は取り除いておく
    x_lowpassValue  = (x_lowpassValue * x_filterCoefficient) + (accel_x * math.cos(y_angle)) * (1 - x_filterCoefficient)
    x_highpassValue = (accel_x * math.cos(y_angle)) - x_lowpassValue
    x_speed         = ((x_highpassValue + (x_oldAccel)) * dt) / 2 + (x_speed)                                           #速度計算(加速度を台形積分する)
    x_oldAccel      = x_highpassValue
    x_difference    = ((((x_speed) + (x_oldSpeed)) * dt) / 2 + x_difference)                                            #変位計算(速度を台形積分する) 
    x_oldSpeed      = x_speed
    #print(x_lowpassValue)
    print("x_highpass",x_highpassValue *1000)
    print(x_difference* 1000)                                                                                           #変位をmmに変換して表示

    z_difference = 0                                                                                                    #変位を初期化　　※ロケット発射時は取り除いておく
    z_lowpassValue  = (z_lowpassValue * z_filterCoefficient) + (accel_z * math.cos(y_angle)) * (1 - z_filterCoefficient)
    z_highpassValue = (accel_z * math.cos(y_angle)) - z_lowpassValue
    z_speed         = ((z_highpassValue + (z_oldAccel)) * dt) / 2 + (z_speed)                                           #速度計算(加速度を台形積分する)
    z_oldAccel      = z_highpassValue
    z_difference    = ((((z_speed) + (z_oldSpeed)) * dt) / 2 + z_difference)                                            #変位計算(速度を台形積分する)
    z_oldSpeed      = z_speed
    #print(z_lowpassValue)
    print("z_highpass",z_highpassValue *1000)
    print(z_difference* 1000)                                                                                           #変位をmmに変換して表示
    
    print("####################################################")
    
    with open('fly_distance.csv', 'a', newline='') as fly_distance_file:                   
        writer = csv.DictWriter(fly_distance_file, fieldnames=fieldnames)  
        writer.writerow({'x_difference':x_difference,'z_difference':z_difference})
        
    time += dt                  #時間を+0.1して値を返す
    if time > calculate_time:   #計測時間
        break
    sleep(dt)                   #0.1秒周期で繰り返す
