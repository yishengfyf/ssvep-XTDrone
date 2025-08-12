#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SSVEP脑电回放与解码模拟器

功能:
1. 无需安装工具箱，内置核心CCA解码算法。
2. 首先发送“起飞/准备”指令。
3. 加载指定路径下的12分类SSVEP数据集。
4. 实时解码EEG数据并发送BUDP无人机移动指令。
"""

import time
import numpy as np
import sys
import os
from scipy.io import loadmat
from scipy.linalg import qr, svd, pinv

# --- 从您师兄的代码中导入 ---
try:
    from budp_sender import BUDPTestSender
except ImportError:
    print("❌ 错误: 找不到 'budp_test_sender.py'。")
    print("👉 请确保此脚本与 budp_test_sender.py 在同一个目录下。")
    sys.exit(1)

# ==============================================================================
#  以下部分是从 ssvep-analysis-toolbox 中提取的核心算法
# ==============================================================================

def canoncorr(X, Y):
    X = np.asarray(X, order='F')
    Y = np.asarray(Y, order='F')
    n, p1 = X.shape
    n_Y, p2 = Y.shape

    if n != n_Y:
        raise ValueError(f"X和Y的行数必须相等 (X有{n}行, Y有{n_Y}行)")

    X -= X.mean(axis=0)
    Y -= Y.mean(axis=0)
    
    Q1, _, _ = qr(X, mode='economic', pivoting=True)
    Q2, _, _ = qr(Y, mode='economic', pivoting=True)
    
    L, d, M_h = svd(Q1.conj().T @ Q2, full_matrices=False)
    M = M_h.conj().T
    
    A = pinv(X) @ Q1 @ L
    B = pinv(Y) @ Q2 @ M
    
    return A, B, d

def gen_ref_sig(freqs, fs, n_points, n_harmonics=5):
    ref_sig = []
    t = np.arange(0, n_points) / fs
    for f in freqs:
        Y = []
        for i in range(1, n_harmonics + 1):
            Y.append(np.sin(2 * np.pi * i * f * t))
            Y.append(np.cos(2 * np.pi * i * f * t))
        ref_sig.append(np.array(Y))
    return ref_sig

class SimpleCCA:
    def __init__(self, n_component=1):
        self.n_component = n_component
        self.ref_sig = None

    def fit(self, ref_sig):
        self.ref_sig = ref_sig

    def predict(self, X_list):
        if self.ref_sig is None:
            raise ValueError("解码器未通过 .fit() 方法进行训练。")
        
        y_pred = []
        for x_single_trial in X_list:
            correlations = []
            for y_ref in self.ref_sig:
                _, _, r = canoncorr(x_single_trial, y_ref.T)
                correlations.append(r[0])
            
            predicted_label = np.argmax(correlations)
            y_pred.append(predicted_label)
            
        return y_pred, None

# ==============================================================================
#  主程序
# ==============================================================================

# --- 1. 参数配置 ---
BCI_BASE_PATH = '~/XTDrone/bci'
DATASET_PATH = os.path.expanduser(os.path.join(BCI_BASE_PATH, 'data'))
SUBJECT_FILENAME = 's3.mat' 

SAMPLING_RATE = 256
ALL_FREQS_IN_DATASET = [9.25, 11.25, 13.25, 9.75, 11.75, 13.75, 10.25, 12.25, 14.25, 10.75, 12.75, 14.75]
STIMULUS_ONSET_SAMPLES = 39

TARGET_FREQS = [9.25, 11.25, 13.25, 9.75, 11.75, 13.75] 
# 注意：BUDP协议中0号是起飞，1-6是移动
COMMAND_MAPPING = {
    9.25: 1, 11.25: 2, 13.25: 3, 9.75: 4, 11.75: 5, 13.75: 6
}
COMMAND_DESC = {
    0: "起飞/准备", 1: "上升", 2: "下降", 3: "前进", 4: "后退", 5: "左转", 6: "右转"
}

WINDOW_LENGTH = 1.0
N_HARMONICS = 5
NUC2_IP = "nuc2"
BUDP_PORT = 20001

# --- 2. 主程序 ---
def main():
    print("🚀 SSVEP脑电回放模拟器 (最终版) 启动")
    sender = BUDPTestSender(target_ip=NUC2_IP, target_port=BUDP_PORT)

    print(f"\n[1/5] 正在加载12-class数据集 (文件: {SUBJECT_FILENAME})...")
    try:
        data_path = os.path.join(DATASET_PATH, SUBJECT_FILENAME)
        mat_data = loadmat(data_path)
        eeg_data = mat_data['eeg']
        print("✅ 数据集加载完成！")
    except FileNotFoundError:
        print(f"❌ 错误：找不到数据集文件 {data_path}")
        sys.exit(1)
    
    print(f"\n[2/5] 正在初始化CCA解码器...")
    window_samples = int(SAMPLING_RATE * WINDOW_LENGTH)
    reference_signals = gen_ref_sig(TARGET_FREQS, SAMPLING_RATE, window_samples, n_harmonics=N_HARMONICS)
    cca_decoder = SimpleCCA(n_component=1)
    cca_decoder.fit(ref_sig=reference_signals)
    print("✅ 解码器初始化完成！")

    # --- 新增步骤：发送起飞指令 ---
    print("\n[3/5] 准备发送初始指令...")
    takeoff_command_id = 0
    print(f"      🚁 发送无人机指令: ID={takeoff_command_id} ({COMMAND_DESC.get(takeoff_command_id, '未知')})")
    sender.send_command(takeoff_command_id, show_debug=False)
    print("      ⏳ 等待5秒，确保无人机进入OFFBOARD并起飞...")
    time.sleep(5.0)
    print("      ✅ 准备阶段完成！")

    print(f"\n[4/5] 开始模拟在线回放与解码...")
    n_trials = eeg_data.shape[3]

    for trial_idx in range(n_trials):
        print(f"\n--- Trial {trial_idx + 1}/{n_trials} ---")
        
        for i, target_freq in enumerate(TARGET_FREQS):
            target_idx_in_dataset = ALL_FREQS_IN_DATASET.index(target_freq)
            trial_data = eeg_data[target_idx_in_dataset, :, :, trial_idx]
            
            print(f"   ▶️ 正在回放目标 {target_freq} Hz 的脑电数据...")
            
            start_sample = STIMULUS_ONSET_SAMPLES
            end_sample = start_sample + window_samples
            eeg_chunk = trial_data[:, start_sample:end_sample].T
            
            predicted_label, _ = cca_decoder.predict([eeg_chunk])
            predicted_freq = TARGET_FREQS[predicted_label[0]]
            
            print(f"      🧠 解码结果: {predicted_freq} Hz")
            
            if abs(predicted_freq - target_freq) < 0.01:
                print(f"      🎯 解码正确！")
                command_id = COMMAND_MAPPING.get(target_freq)
                if command_id is not None:
                    print(f"      🚁 发送无人机指令: ID={command_id} ({COMMAND_DESC.get(command_id, '未知')})")
                    sender.send_command(command_id, show_debug=False)
            else:
                print(f"      ❌ 解码错误 (预期: {target_freq} Hz)")
            
            time.sleep(WINDOW_LENGTH + 6.0)

    print(f"\n[5/5] 所有Block回放完毕！")
    sender.close()

if __name__ == "__main__":
    main()