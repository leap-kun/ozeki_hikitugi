from scipy.ndimage import convolve
from scipy.signal import butter, sosfreqz, sosfilt
from scipy.ndimage import median_filter
from pykalman import KalmanFilter 
import math
import numpy as np

from collections import deque
import statistics



#指数移動平均
def calculate_ema(data, alpha):
    ema = [data[0]]  # 初期値は最初のデータポイントと同じ
    for i in range(1, len(data)):
        ema_value = alpha * data[i] + (1 - alpha) * ema[i-1]
        ema.append(ema_value)
    return np.array(ema)


def apply_moving_average_3d(data, window_size):
    """
    3次元座標ごとに移動平均を取る関数。

    パラメータ:
    - data: 移動平均を取る対象の3次元座標の配列 (3×Nの2次元配列)。
    - window_size: 移動平均のウィンドウサイズ。

    戻り値:
    - smoothed_data: 移動平均を取った後の配列。
    """

    def valid_convolve(xx, size):
        """
        移動平均を取る関数。

        パラメータ:
        - xx: 移動平均を取る対象の配列。
        - size: 移動平均のウィンドウサイズ。

        戻り値:
        - xx_mean: 移動平均を取った後の配列。
        """
        
        # 移動平均フィルタを作成
        b = np.ones(size) / size
        # 移動平均を計算
        xx_mean = np.convolve(xx, b, mode="same")

        # 補正部分の計算
        n_conv = math.ceil(size / 2)
        # 端の部分の補正
        xx_mean[0] *= size / n_conv
        for i in range(1, n_conv):
            xx_mean[i] *= size / (i + n_conv)
            xx_mean[-i] *= size / (i + n_conv - (size % 2))
            # size % 2は奇数偶数での違いに対応するため

        return xx_mean

    # 各座標ごとに移動平均を計算
    smoothed_data = np.array([valid_convolve(data[:, i], window_size) for i in range(data.shape[1])]).T

    return smoothed_data

def gaussian_filter(data, sigma=1):
    """
    ガウス畳み込みによるローパスフィルターを適用する関数

    Parameters:
    - data: 3x1の座標データ (NumPy配列)
    - sigma: ガウス分布の標準偏差

    Returns:
    - ローパスフィルターを適用した座標データ (NumPy配列)
    """
    # ガウス関数の定義
    def gaussian(x, sigma):
        return np.exp(-x**2 / (2 * sigma**2)) / (np.sqrt(2 * np.pi) * sigma)

    # カーネルのサイズを定義
    kernel_size = int(6 * sigma + 1)
    if kernel_size % 2 == 0:
        kernel_size += 1  # カーネルサイズが奇数になるように調整

    # ガウスカーネルの作成
    kernel = gaussian(np.arange(kernel_size) - (kernel_size - 1) / 2, sigma)

    # 正規化
    kernel /= np.sum(kernel)

    # 畳み込みを行う
    result = convolve(data, kernel.reshape(-1, 1))

    return result

def exponential_moving_average(ema_prev, new_data, alpha):
    """
    指数移動平均を計算する関数

    Parameters:
    - ema_prev: 前の時点の指数移動平均 (numpy array, shape: (3, 1))
    - new_data: 新しいデータ点 (numpy array, shape: (3, 1))
    - alpha: 平滑化係数 (0 < alpha < 1)

    Returns:
    - ema: 新しい指数移動平均 (numpy array, shape: (3, 1))
    """
    ema = alpha * new_data + (1 - alpha) * ema_prev
    return ema

def apply_butterworth_filter_3d(data, cutoff_freq, order=4, fs=1.0):
    """
    3次元座標ごとにバタワースフィルタを適用する関数。

    パラメータ:
    - data: バタワースフィルタを適用する対象の3次元座標の配列 (3×Nの2次元配列)。
    - cutoff_freq: カットオフ周波数 (Hz)。
    - order: フィルタの次数。
    - fs: サンプリング周波数 (Hz)。

    戻り値:
    - filtered_data: バタワースフィルタを適用した後の座標。
    """

    # データ点の数
    num_points = data.shape[0]

    # バタワースフィルタの設計
    sos = butter(order, cutoff_freq, btype='low', fs=fs, output='sos')

    # フィルタの周波数応答
    _, h = sosfreqz(sos, worN=num_points, fs=fs)

    # 3次元座標ごとにフィルタを適用
    filtered_data = np.array([sosfilt(sos, data[:, i]) for i in range(data.shape[1])]).T

    return filtered_data

def valid_convolve(xx, size):
    b = np.ones(size)/size
    xx_mean = np.convolve(xx, b, mode="same")

    n_conv = math.ceil(size/2)

    # 補正部分
    xx_mean[0] *= size/n_conv
    for i in range(1, n_conv):
        xx_mean[i] *= size/(i+n_conv)
        xx_mean[-i] *= size/(i + n_conv - (size % 2)) 
	# size%2は奇数偶数での違いに対応するため

    return xx_mean


def median1d(arr, k):
    w = len(arr)
    half_k = k // 2
    result = np.zeros_like(arr, dtype=np.float64)

    for i in range(w):
        start = max(0, i - half_k)
        end = min(w, i + half_k + 1)
        window = arr[start:end]
        result[i] = np.median(window)

    return result

"""
def kalman_filter(measurements):
    initial_state = 1.75  # 初期状態の推定値
    initial_estimate_error = 0.1  # 初期推定誤差の共分散行列
    # システム行列の設定
    A = np.array([[1]])
    # 観測行列の設定
    H = np.array([[1]])
    # システムノイズの共分散行列の設定
    Q = np.array([[0.01]])
    # 観測ノイズの共分散行列の設定
    R = np.array([[0.1]])
    # カルマンフィルタの定義
    kf = KalmanFilter(initial_state_mean=initial_state,
                      initial_state_covariance=initial_estimate_error,
                      transition_matrices=A,
                      observation_matrices=H,
                      transition_covariance=Q,
                      observation_covariance=R)
    # カルマンフィルタの適用
    state_means, _ = kf.filter(np.array(measurements))
    
    state_means = np.asarray(state_means)
    
    return state_means
"""
def kalman_filter(measurements):
    initial_state = 1.75  # 初期状態の推定値
    initial_estimate_error = 0.1  # 初期推定誤差の共分散行列

    # システム行列の設定
    A = np.array([[1]])
    # 観測行列の設定
    H = np.array([[1]])
    # システムノイズの共分散行列の設定
    Q = np.array([[0.1]])
    # 観測ノイズの共分散行列の設定
    R = np.array([[0.1]])

    # カルマンフィルタの初期化
    state_mean = np.array([initial_state])
    state_covariance = np.array([initial_estimate_error])

    # 結果保存用のリスト
    filtered_states = []

    # カルマンフィルタの適用
    for measurement in measurements:
        # 予測ステップ
        predicted_state_mean = A.dot(state_mean)
        predicted_state_covariance = A.dot(state_covariance).dot(A.T) + Q

        # 更新ステップ
        kalman_gain = predicted_state_covariance.dot(H.T).dot(np.linalg.inv(H.dot(predicted_state_covariance).dot(H.T) + R))
        state_mean = predicted_state_mean + kalman_gain.dot(measurement - H.dot(predicted_state_mean))
        state_covariance = (1 - kalman_gain.dot(H)).dot(predicted_state_covariance)

        # 結果を保存
        filtered_states.append(state_mean[0])

    return np.array(filtered_states)
"""
# 使用例
measurements = np.array([1.7, 1.84, 2.3, 1.7, 1.9])  # 観測値のnumpy配列
# カルマンフィルタの実行
state_estimates = kalman_filter(measurements)
# 結果の表示
print("State estimates:", state_estimates[-1])
"""