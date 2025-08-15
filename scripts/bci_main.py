#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SSVEP BCI 综合主控程序

功能:
1. 显示SSVEP刺激界面。
2. 在后台回放并解码真实的EEG数据集。
3. 将解码结果实时显示在界面上作为反馈。
4. 将解码指令通过UDP发送给无人机仿真。
5. 内置核心CCA解码算法，无需外部工具箱。
"""

import os
import pygame
import sys
import time
import numpy as np
from scipy.io import loadmat
from scipy.linalg import qr, svd, pinv
from budp_sender import BUDPTestSender
from ssvep_sender import gen_ref_sig, SimpleCCA

# --- 禁用Pygame音频模块 ---
os.environ['SDL_AUDIODRIVER'] = 'dummy'

# ==============================================================================
#  Pygame界面与主程序
# ==============================================================================

# --- 1. 参数配置 ---
pygame.init()
pygame.font.init()

WIDTH, HEIGHT = 1200, 800
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("SSVEP 脑控界面 ")

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
square_size = 160
margin_x = 50
margin_y = 40
gap_y = (HEIGHT - 2 * margin_y - 3 * square_size) / 2
FPS = 60

try:
    font_path = "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc"
    font_stimulus = pygame.font.Font(font_path, 36)
    font_feedback = pygame.font.Font(font_path, 52)
except FileNotFoundError:
    font_stimulus = pygame.font.SysFont("arial", 36)
    font_feedback = pygame.font.SysFont("arial", 52)

# --- 数据集与解码参数 注意数据路径---
DATASET_PATH = os.path.expanduser('~/XTDrone/bci/ssvep-XTDrone-k/data')
SUBJECT_FILENAME = 's1.mat' 
SAMPLING_RATE = 256
ALL_FREQS_IN_DATASET = [9.25, 11.25, 13.25, 9.75, 11.75, 13.75, 10.25, 12.25, 14.25, 10.75, 12.75, 14.75]
STIMULUS_ONSET_SAMPLES = 39

TARGET_FREQS = [9.25, 11.25, 13.25, 9.75, 11.75, 13.75] 
COMMAND_MAPPING = {
    9.25: 1, 11.25: 2, 13.25: 3, 9.75: 4, 11.75: 5, 13.75: 6
}
COMMAND_DESC = {
    0: "起飞/准备", 1: "上升", 2: "下降", 3: "前进", 4: "后退", 5: "左移", 6: "右移"
}

WINDOW_LENGTH = 1.0
N_HARMONICS = 5
NUC2_IP = "172.20.10.108"
BUDP_PORT = 20001

# --- 刺激块数据与布局 ---
stimuli_data = [
    (COMMAND_MAPPING[9.25], "上升", 9.25),
    (COMMAND_MAPPING[11.25], "下降", 11.25),
    (COMMAND_MAPPING[13.25], "前进", 13.25),
    (COMMAND_MAPPING[9.75], "后退", 9.75),
    (COMMAND_MAPPING[11.75], "左移", 11.75),
    (COMMAND_MAPPING[13.75], "右移", 13.75),
]
positions = [
    (margin_x, margin_y), (WIDTH - square_size - margin_x, margin_y),
    (margin_x, margin_y + square_size + gap_y), (WIDTH - square_size - margin_x, margin_y + square_size + gap_y),
    (margin_x, margin_y + 2 * (square_size + gap_y)), (WIDTH - square_size - margin_x, margin_y + 2 * (square_size + gap_y)),
]
stimuli = []
for i, (cmd_id, text, freq) in enumerate(stimuli_data):
    x, y = positions[i]
    stimuli.append({
        'freq': freq, 'text': text, 'command_id': cmd_id,
        'rect': pygame.Rect(x, y, square_size, square_size),
        'period_frames': FPS / freq if freq != 0 else float('inf'),
    })

# --- 主程序 ---
def main():
    print("SSVEP BCI 综合主控程序启动")
    
    sender = BUDPTestSender(target_ip=NUC2_IP, target_port=BUDP_PORT)

    print("\n[1/4] 正在加载数据集...")
    try:
        data_path = os.path.join(DATASET_PATH, SUBJECT_FILENAME)
        mat_data = loadmat(data_path)
        eeg_data = mat_data['eeg']
        print("数据集加载完成！")
    except FileNotFoundError:
        print(f"错误：找不到数据集文件 {data_path}")
        sys.exit(1)
    
    print("\n[2/4] 正在初始化CCA解码器...")
    window_samples = int(SAMPLING_RATE * WINDOW_LENGTH)
    reference_signals = gen_ref_sig(TARGET_FREQS, SAMPLING_RATE, window_samples, n_harmonics=N_HARMONICS)
    cca_decoder = SimpleCCA(n_component=1)
    cca_decoder.fit(ref_sig=reference_signals)
    print("解码器初始化完成！")

    print("\n[3/4] 准备发送初始指令...")
    while not sender.send_command(0, show_debug=False):
        print("      起飞指令发送失败，正在重试...")
        time.sleep(1) # 等待1秒后重试
    # sender.send_command(0, show_debug=False)
    print("      发送无人机指令: ID=0 (起飞/准备)")
    print(f"      等待5秒...")
    time.sleep(5.0)
    print("      准备阶段完成！")

    print("\n[4/4] 进入主循环，开始实时回放与控制...")
    clock = pygame.time.Clock()
    frame_counter = 0
    feedback_message = ""
    feedback_timer = 0
    
    n_trials = eeg_data.shape[3]
    current_trial_idx = 0
    current_target_idx = 0
    
    playback_interval_frames = int(FPS * 6)
    playback_timer = playback_interval_frames

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                running = False
        
        playback_timer -= 1
        if playback_timer <= 0:
            target_freq = TARGET_FREQS[current_target_idx]
            target_idx_in_dataset = ALL_FREQS_IN_DATASET.index(target_freq)
            
            trial_data = eeg_data[target_idx_in_dataset, :, :, current_trial_idx]
            start_sample = STIMULUS_ONSET_SAMPLES
            end_sample = start_sample + window_samples
            eeg_chunk = trial_data[:, start_sample:end_sample].T
            
            predicted_label, _ = cca_decoder.predict([eeg_chunk])
            predicted_freq = TARGET_FREQS[predicted_label[0]]
            
            print(f"\n回放 Trial {current_trial_idx+1}, 目标 {target_freq} Hz -> 解码结果: {predicted_freq} Hz")
            
            if abs(predicted_freq - target_freq) < 0.01:
                command_id = COMMAND_MAPPING.get(target_freq)
                command_text = COMMAND_DESC.get(command_id, "未知")
                
                print(f"      解码正确！发送指令: ID={command_id} ({command_text})")
                
                feedback_message = f"识别控制指令：{command_text}"
                feedback_timer = int(FPS * 2)
                while not sender.send_command(command_id, show_debug=False):
                    print(f"      指令 {command_id} 发送失败，正在重试...")
                    time.sleep(1) # 等待1秒后重试
                print(f"      指令 {command_id} 发送成功!")
                # sender.send_command(command_id, show_debug=False)
            else:
                print(f"      解码错误")
            
            current_target_idx += 1
            if current_target_idx >= len(TARGET_FREQS):
                current_target_idx = 0
                current_trial_idx += 1
                if current_trial_idx >= n_trials:
                    print("\n所有数据回放完毕！程序将在5秒后退出。")
                    time.sleep(5)
                    running = False
            
            playback_timer = playback_interval_frames

        screen.fill(BLACK)
        if feedback_timer > 0:
            for stimulus in stimuli:
                pygame.draw.rect(screen, WHITE, stimulus['rect'])
                text_surface = font_stimulus.render(stimulus['text'], True, WHITE)
                text_rect = text_surface.get_rect(center=stimulus['rect'].center)
                screen.blit(text_surface, text_rect)
            feedback_surface = font_feedback.render(feedback_message, True, RED)
            feedback_rect = feedback_surface.get_rect(center=(WIDTH / 2, HEIGHT / 2))
            screen.blit(feedback_surface, feedback_rect)
            feedback_timer -= 1
        else:
            for stimulus in stimuli:
                half_period = stimulus['period_frames'] / 2
                is_on = (frame_counter % stimulus['period_frames']) < half_period if half_period > 0 else True
                color = WHITE if is_on else BLACK
                pygame.draw.rect(screen, color, stimulus['rect'])
                text_surface = font_stimulus.render(stimulus['text'], True, WHITE)
                text_rect = text_surface.get_rect(center=stimulus['rect'].center)
                screen.blit(text_surface, text_rect)
        
        pygame.display.flip()
        frame_counter += 1
        clock.tick(FPS)

    sender.close()
    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()