import pandas as pd
import matplotlib.pyplot as plt
import ast
import numpy as np

def kalman_filter(measurements):
    initial_state = 1.75  # 初期状態の推定値
    initial_estimate_error = 0.1  # 初期推定誤差の共分散行列

    A = np.array([[1]])  # システム行列の設定
    H = np.array([[1]])  # 観測行列の設定
    Q = np.array([[0.1]])  # システムノイズの共分散行列の設定
    R = np.array([[0.1]])  # 観測ノイズの共分散行列の設定

    state_mean = np.array([initial_state])  # カルマンフィルタの初期化
    state_covariance = np.array([initial_estimate_error])

    filtered_states = []  # 結果保存用のリスト

    for measurement in measurements:
        predicted_state_mean = A.dot(state_mean)
        predicted_state_covariance = A.dot(state_covariance).dot(A.T) + Q

        kalman_gain = predicted_state_covariance.dot(H.T).dot(np.linalg.inv(H.dot(predicted_state_covariance).dot(H.T) + R))
        state_mean = predicted_state_mean + kalman_gain.dot(measurement - H.dot(predicted_state_mean))
        state_covariance = (1 - kalman_gain.dot(H)).dot(predicted_state_covariance)

        filtered_states.append(state_mean[0])

    return np.array(filtered_states)

def delete_columns_and_plot(input_csv, output_csv, columns_to_delete):
    df = pd.read_csv(input_csv)

    for column in columns_to_delete:
        if column in df.columns:
            df = df.drop(columns=[column])
            print(f"列 '{column}' を正常に削除しました。")
        else:
            print(f"列 '{column}' が見つかりませんでした。")

    df.to_csv(output_csv, index=False)
    print(f"新しいCSVファイル '{output_csv}' を正常に作成しました。")

    if 'time' in df.columns:
        df['time'] = pd.to_datetime(df['time'], errors='coerce')

        if '.data' in df.columns:
            df['.data'] = df['.data'].apply(ast.literal_eval)

            df['.data'] = df['.data'].apply(lambda x: kalman_filter(x))

            df['seconds'] = (df['time'] - df['time'].min()).dt.total_seconds()

            plt.plot(df['seconds'], df['.data'].apply(lambda x: x[0]), label='ex')
            plt.plot(df['seconds'], df['.data'].apply(lambda x: x[1]), label='ey')
            plt.plot(df['seconds'], df['.data'].apply(lambda x: x[2]), label='ez')

            plt.xlabel('時間[s]')
            plt.ylabel('ez')
            plt.title('ezのデータ')
            plt.legend(loc='upper right')
            plt.tight_layout()
            plt.show()
        else:
            print("エラー: '.data'列がDataFrameに存在しません。")
    else:
        print("エラー: 'time'列がDataFrameに存在しません。")

# 使用例
input_file = '/home/kajilab/catkin_ws/src/rosbag_to_csv/scripts/Pub_point_2024_01_11_19:25point_array.csv'
output_file = '/home/kajilab/catkin_ws/src/rosbag_to_csv/scripts/test_remake.csv'
columns_to_remove = ['.layout.dim', '.layout.data_offset']

delete_columns_and_plot(input_file, output_file, columns_to_remove)
