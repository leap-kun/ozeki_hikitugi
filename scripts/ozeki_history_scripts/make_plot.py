import pandas as pd
import matplotlib.pyplot as plt
import ast  # タプルの文字列表現を解析するためのモジュール

# CSV ファイルを読み込む
df = pd.read_csv('/home/kajilab/catkin_ws/src/rosbag_to_csv/scripts/test_remake.csv', delimiter=',')

# 'time'列が存在するか確認
if 'time' in df.columns:
    # 'time' 列を直接 datetime に変換する
    df['time'] = pd.to_datetime(df['time'], errors='coerce')

    # '.data'列が存在するか確認
    if '.data' in df.columns:
        # '.data' 列の文字列形式のタプルを実際のタプルに変換
        df['.data'] = df['.data'].apply(ast.literal_eval)

        # 0秒からの経過時間に変換
        df['seconds'] = (df['time'] - df['time'].min()).dt.total_seconds()

        # グラフを描画する
        plt.plot(df['seconds'], df['.data'].apply(lambda x: x[0]), label='ex')
        plt.plot(df['seconds'], df['.data'].apply(lambda x: x[1]), label='ey')
        plt.plot(df['seconds'], df['.data'].apply(lambda x: x[2]), label='ez')

        plt.xlabel('time[s]')
        plt.ylabel('ez')
        plt.title('Data of ez')
        plt.legend(loc='upper right')  # 凡例を右上に表示
        plt.show()
    else:
        print("エラー: '.data'列がDataFrameに存在しません。")
else:
    print("エラー: 'time'列がDataFrameに存在しません。")
