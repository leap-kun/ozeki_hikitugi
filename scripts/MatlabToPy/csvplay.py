import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation as R

def make_ez_from_csv(csv_filename):
    # CSVファイルからデータを読み込む
    data = np.genfromtxt(csv_filename, delimiter=',', skip_header=1)  # ヘッダーがある場合はスキップする

    # 時間とezデータを抽出
    time = data[:, 0]
    ez_data = data[:, 1:4]

    # 非有限な値を含む行を確認および削除
    valid_rows = np.all(np.isfinite(ez_data), axis=1)
    time = time[valid_rows]
    ez_data = ez_data[valid_rows]

    # ez_dataの形状を元のコードで使用されている形状に変換
    ez_data = ez_data.T

    # 必要に応じてez_dataをラジアンに変換
    ez_data = np.radians(ez_data)

    # コードの他の部分は変更しないものとしています
    q = cubic_poly_traj(ez_data, time)

    if q is None:
        return None, None, None

    tsize = len(time)
    ez = np.zeros((3, tsize))

    # 肩の運動学
    Rz = R.from_euler('XYZ', [0, 0, np.pi])

    for n in range(tsize):
        q1 = q[0, n]
        q2 = q[1, n]
        Ry = R.from_euler('XYZ', [0, q1, 0])
        Rx = R.from_euler('XYZ', [q2, 0, 0])
        R_temp = Rz * Ry * Rx
        ez[:, n] = R_temp.apply(np.array([0, 0, 1]))

    return time, q, ez

def cubic_poly_traj(Pq, time):
    Tq = np.arange(len(Pq[0]))

    if len(Tq) < 2:
        print("警告: `Tq` の要素数が少なすぎます。")

    cs1 = CubicSpline(Tq, Pq[0])
    cs2 = CubicSpline(Tq, Pq[1])

    q1 = cs1(time)
    q2 = cs2(time)

    q = np.vstack((q1, q2))

    return q

# 例としての使用法:
csv_filename = 'test.csv'  # 実際のCSVファイル名に置き換えてください
time, q, ez = make_ez_from_csv(csv_filename)
