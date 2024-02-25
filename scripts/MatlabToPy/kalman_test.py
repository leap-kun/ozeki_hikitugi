import numpy as np
import matplotlib.pyplot as plt

def first_order_lowpass_filter(input_signal, time_constant, sampling_period):
    """
    1次ローパスフィルタを適用する関数

    Parameters:
    - input_signal: 入力信号の配列 (N×1のNumPy配列)
    - time_constant: ローパスフィルタの時定数
    - sampling_period: サンプリング周期

    Returns:
    - filtered_signal: フィルタリングされた信号の配列 (N×1のNumPy配列)
    """

    alpha = time_constant / (time_constant + sampling_period)
    filtered_signal = np.zeros_like(input_signal)

    for i in range(1, len(input_signal)):
        filtered_signal[i] = alpha * input_signal[i] + (1 - alpha) * filtered_signal[i - 1]

    return filtered_signal

# テスト用の信号生成
time = np.arange(0, 5, 0.01)
input_signal = np.sin(2 * np.pi * 1 * time).reshape(-1, 1) + 0.5 * np.random.normal(size=(len(time), 1))

# ローパスフィルタのパラメータ
time_constant = 0.1
sampling_period = time[1] - time[0]

# 1次ローパスフィルタの適用
filtered_signal = first_order_lowpass_filter(input_signal, time_constant, sampling_period)

# 結果のプロット
plt.figure(figsize=(10, 6))
plt.plot(time, input_signal, label='Input Signal')
plt.plot(time, filtered_signal, label='Filtered Signal')
plt.title('1st Order Lowpass Filter')
plt.xlabel('Time')
plt.ylabel('Amplitude')
plt.legend()
plt.show()
