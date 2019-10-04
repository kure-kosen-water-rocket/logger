import smbus            # use I2C
import math             # mathmatics
from time import sleep  # time module
from smbus import SMBus

import RPi.GPIO as GPIO
import signal
import sys
import csv
import matplotlib
matplotlib.use('Agg')
import pandas as pd
import matplotlib.pyplot as plt

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

class Bme280:                          #BME280初期設定
    def __init__(self, busNumber=1, i2cAddress=0x76):
        self.bus = SMBus(busNumber)
        self.i2cAddress = i2cAddress
        self.digT = []
        self.digP = []
        self.digH = []
        self.timeFine = 0.0
        self.presRaw  = 0.0
        self.tempRaw  = 0.0
        self.humRaw   = 0.0
        osrsT   = 1         #Temperature oversampling x 1
        osrsP   = 1         #Pressure oversampling x 1
        osrsH   = 1         #Humidity oversampling x 1
        mode    = 3         #Normal mode
        tSb     = 5         #Tstandby 1000ms
        filter  = 0         #Filter off
        spi3wEn = 0         #3-wire SPI Disable
        ctrlMeasReg = (osrsT << 5) | (osrsP << 2) | mode
        configReg   = (tSb << 5) | (filter << 2) | spi3wEn
        ctrlHumReg  = osrsH
        self.writeReg(0xF2,ctrlHumReg)
        self.writeReg(0xF4,ctrlMeasReg)
        self.writeReg(0xF5,configReg)
        self.getCalibParam()
        self.readData()

    def writeReg(self, regAddress, data):
        self.bus.write_byte_data(self.i2cAddress, regAddress, data)

    def getCalibParam(self):
        calib = []
        for i in range (0x88,0x88+24):
            calib.append(self.bus.read_byte_data(self.i2cAddress,i))
        calib.append(self.bus.read_byte_data(self.i2cAddress,0xA1))
        for i in range (0xE1,0xE1+7):
            calib.append(self.bus.read_byte_data(self.i2cAddress,i))

        self.digT.append((calib[1] << 8) | calib[0])
        self.digT.append((calib[3] << 8) | calib[2])
        self.digT.append((calib[5] << 8) | calib[4])
        self.digP.append((calib[7] << 8) | calib[6])
        self.digP.append((calib[9] << 8) | calib[8])
        self.digP.append((calib[11]<< 8) | calib[10])
        self.digP.append((calib[13]<< 8) | calib[12])
        self.digP.append((calib[15]<< 8) | calib[14])
        self.digP.append((calib[17]<< 8) | calib[16])
        self.digP.append((calib[19]<< 8) | calib[18])
        self.digP.append((calib[21]<< 8) | calib[20])
        self.digP.append((calib[23]<< 8) | calib[22])
        self.digH.append( calib[24] )
        self.digH.append((calib[26]<< 8) | calib[25])
        self.digH.append( calib[27] )
        self.digH.append((calib[28]<< 4) | (0x0F & calib[29]))
        self.digH.append((calib[30]<< 4) | ((calib[29] >> 4) & 0x0F))
        self.digH.append( calib[31] )

        for i in range(1,2):
            if self.digT[i] & 0x8000:
                self.digT[i] = (-self.digT[i] ^ 0xFFFF) + 1
        for i in range(1,8):
            if self.digP[i] & 0x8000:
                self.digP[i] = (-self.digP[i] ^ 0xFFFF) + 1
        for i in range(0,6):
            if self.digH[i] & 0x8000:
                self.digH[i] = (-self.digH[i] ^ 0xFFFF) + 1  

    def readData(self):
        data = []
        for i in range (0xF7, 0xF7+8):
            data.append(self.bus.read_byte_data(self.i2cAddress,i))

        self.presRaw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        self.tempRaw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        self.humRaw  = (data[6] << 8)  |  data[7]

    def getPressure(self):
        pressure = 0.0
        v1 = (self.timeFine / 2.0) - 64000.0
        v2 = (((v1 / 4.0) * (v1 / 4.0)) / 2048) * self.digP[5]
        v2 = v2 + ((v1 * self.digP[4]) * 2.0)
        v2 = (v2 / 4.0) + (self.digP[3] * 65536.0)
        v1 = (((self.digP[2] * (((v1 / 4.0) * (v1 / 4.0)) / 8192)) / 8)  + ((self.digP[1] * v1) / 2.0)) / 262144
        v1 = ((32768 + v1) * self.digP[0]) / 32768

        if v1 == 0:
            return 0
        pressure = ((1048576 - self.presRaw) - (v2 / 4096)) * 3125
        if pressure < 0x80000000:
            pressure = (pressure * 2.0) / v1
        else:
            pressure = (pressure / v1) * 2
        v1 = (self.digP[8] * (((pressure / 8.0) * (pressure / 8.0)) / 8192.0)) / 4096
        v2 = ((pressure / 4.0) * self.digP[7]) / 8192.0
        pressure = pressure + ((v1 + v2 + self.digP[6]) / 16.0)
        return pressure/100

    def getTemperature(self):
        v1 = (self.tempRaw / 16384.0 - self.digT[0] / 1024.0) * self.digT[1]
        v2 = (self.tempRaw / 131072.0 - self.digT[0] / 8192.0) * (self.tempRaw / 131072.0 - self.digT[0] / 8192.0) * self.digT[2]
        self.timeFine = v1 + v2
        temperature = self.timeFine / 5120.0
        return temperature

    def getHumidity(self):
        varH = self.timeFine - 76800.0
        if varH != 0:
            varH = (self.humRaw - (self.digH[3] * 64.0 + self.digH[4]/16384.0 * varH)) * (self.digH[1] / 65536.0 * (1.0 + self.digH[5] / 67108864.0 * varH * (1.0 + self.digH[2] / 67108864.0 * varH)))
        else:
            return 0
        varH = varH * (1.0 - self.digH[0] * varH / 524288.0)
        if varH > 100.0:
            varH = 100.0
        elif varH < 0.0:
            varH = 0.0
        return varH


#----------------------------------------------------------------------------------------------------------------------------------------------------------------
dt            = 0.1
filterCoefficient = 0.5  #ローパスフィルターの係数(これは環境によって要調整。1に近づけるほど平滑化の度合いが大きくなる。
lowpassValue  = 0
highpassValue = 0
speed         = 0        #加速度時から算出した速度
oldSpeed      = 0        #ひとつ前の速度
oldAccel      = 0        #ひとつ前の加速度
time          = 0
x_data        = []       #時間のリスト
y_data        = []       #温度のリスト

while 1:
    sensor = Bme280()    #BME280インスタンス化
    
    #print("気圧 : %7.2f hPa" % sensor.getPressure())                                       #気圧、温度、標高表示
    #print("温度 :  %-6.2f ℃" % sensor.getTemperature())
    h = (((1013.25 / sensor.getPressure())**(1 / 5.257)-1)*( sensor.getTemperature()+273.15)) / 0.0065
    #print(h)

    gyro_x,gyro_y,gyro_z = get_gyro_data_deg()                                              #角速度表示
    #print ('gyro[deg/s]')
    #print ('x: %08.3f' % gyro_x)
    #print ('y: %08.3f' % gyro_y)
    #print ('z: %08.3f' % gyro_z)
    #print ('||')
    
    accel_x,accel_y,accel_z = get_accel_data_g()                                            #加速度表示
    #print ('accel[g]')
    #print ('x: %06.3f' % accel_x)
    #print ('y: %06.3f' % accel_y)
    #print ('z: %06.3f' % accel_z)
    
    y_angle = math.degrees(math.atan2(accel_x , math.sqrt(accel_y**2 + accel_z**2)))        #姿勢角の算出
    y_angle = 0.995 * (y_angle + gyro_y * dt) + (0.005* accel_y)
    #print(round(y_angle,0) ,"度")
    #y_angle = math.radians(y_angle)
    
    difference = 0                                                                          #変位を初期化　　※ロケット発射時は取り除いておく
    lowpassValue = (lowpassValue * filterCoefficient) + (accel_x) * (1 - filterCoefficient) #ローパスフィルター(現在の値 = 係数 * ひとつ前の値 ＋ センサの値 * (1 - 係数))
    highpassValue = (accel_x) - lowpassValue                                                #ハイパスフィルター(加速度の値-ローパス値)
    #print(highpassValue)

    speed = ((highpassValue + (oldAccel)) * dt) / 2 + (speed)                               #速度計算(加速度を台形積分する)
    oldAccel = highpassValue
    difference = ((((speed * math.cos(y_angle)) + (oldSpeed * math.cos(y_angle))) * dt) / 2 + difference) #変位計算(速度を台形積分する)
    oldSpeed = speed
    #print(difference*1000) 　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　 #変位をmmに変換して表示
    
    print("####################################################")
    
    temperature = round(sensor.getTemperature(), 1)                                         #BME280から温度を取得
    time += 0.5                                                                             #時間を+0.5して値を返す
    x_data.append(time)                                                                     #x_dataのリストに温度を格納する
    y_data.append(temperature)                                                              #y_dataのリストに時間を格納する
    print(temperature)

    if time > 30.0:                                                                         #30s
        break
    
    sleep(0.5)                                                                              #0.5秒周期で繰り返す

plt.plot(x_data, y_data, label="temperature")
plt.savefig('temperature.png')