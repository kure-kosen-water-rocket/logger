import pandas as pd
import csv
import math

df = pd.read_csv("measurement.csv")

i = 0
z_old_accel = 0
x_old_accel = 0
z_old_speed = 0
x_old_speed = 0
dt = 0.01
fieldnames = ['x_distance', 'z_distance']

for i in range(df.shape[0]):
    z_acceleration = df.iloc[i,2]
    x_acceleration = df.iloc[i,0]
    y_angle = df.iloc[i,3]
    z_g_acceleration = z_acceleration * math.cos(y_angle)
    x_g_acceleration = x_acceleration * math.cos(y_angle)

    z_m_acceleration = z_acceleration - z_g_acceleration
    x_m_acceleration = x_acceleration - x_g_acceleration

    z_s_acceleration = z_m_acceleration * math.cos(y_angle)
    x_s_acceleration = x_m_acceleration * math.cos(y_angle)

    z_speed = (((z_s_acceleration + z_old_accel) * dt) /2)
    x_speed = (((x_s_acceleration + x_old_accel) * dt)/2)

    z_distance = (((z_speed + z_old_speed) * dt) / 2)
    x_distance = (((x_speed + x_old_speed) * dt) / 2)

    z_old_accel = z_s_acceleration
    x_old_accel = x_s_acceleration

    z_old_speed = z_speed
    x_old_speed = z_speed

    with open('fly_distance.csv', 'a', newline='') as fly_distance_file:
        writer = csv.DictWriter(fly_distance_file, fieldnames=fieldnames)
        writer.writerow({'z_distance':z_distance,'x_distance':x_distance})