import csv
import matplotlib
matplotlib.use('Agg')
import pandas as pd
import matplotlib.pyplot as plt

def read_csv():
    df = pd.read_csv('fly_distance.csv')           #fly_distance.csvを'df'という名前で読み込む
    x_data = list(df.iloc[:, 0])                   #dfの0列目の全てのデータをlist型でx_data[]に収める
    y_data = list(df.iloc[:, 1])                   #dfの1列目の全てのデータをlist型でy_data[]に収める

def graph_plot():
    plt.plot(x_data, y_data, label="displacement") #グラフのx軸、y軸をx_data(x_difference),y_data(z_difference)に設定
    plt.title('fly_distance')                      #タイトルを'fly_distance'に設定
    plt.x_label('x_difference')                    #x軸の名前を'x_difference'に設定
    plt.y_label('z_difference')                    #y軸の名前を'z_difference'に設定
    plt.grid()                                     #罫線の追加
    plt.savefig('fly_distance.png')                #グラフを'fly_distance.png'という名前で保存

read_csv()
graph_plot()