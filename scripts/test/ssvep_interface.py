#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import pygame
import sys

# --- 关键修复：在Pygame初始化前，禁用音频驱动 ---
os.environ['SDL_AUDIODRIVER'] = 'dummy'

# --- 1. 初始化和参数设置 ---
pygame.init()
pygame.font.init()

WIDTH, HEIGHT = 1200, 800
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("SSVEP 脑控界面")

# 颜色定义
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
# GREEN = (0, 255, 0) # 不再需要绿色

# 刺激块参数
square_size = 160
margin_x = 50
margin_y = 40
gap_y = (HEIGHT - 2 * margin_y - 3 * square_size) / 2
FPS = 60

# --- 2. 中文字体支持 ---
try:
    font_path = "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc"
    font_stimulus = pygame.font.Font(font_path, 36)
    font_feedback = pygame.font.Font(font_path, 52)
except FileNotFoundError:
    print("警告：未找到中文字体，将使用默认字体，中文可能显示为方块。")
    font_stimulus = pygame.font.SysFont("arial", 36)
    font_feedback = pygame.font.Font(font_path, 52)

# --- 3. 刺激块数据与新布局 ---
stimuli_data = [
    (1, "上升", pygame.K_1),
    (2, "下降", pygame.K_2),
    (4, "前进", pygame.K_3),
    (8, "后退", pygame.K_4),
    (16, "左移", pygame.K_5),
    (32, "右移", pygame.K_6),
]

positions = [
    (margin_x, margin_y),
    (WIDTH - square_size - margin_x, margin_y),
    (margin_x, margin_y + square_size + gap_y),
    (WIDTH - square_size - margin_x, margin_y + square_size + gap_y),
    (margin_x, margin_y + 2 * (square_size + gap_y)),
    (WIDTH - square_size - margin_x, margin_y + 2 * (square_size + gap_y)),
]

stimuli = []
for i, (freq, text, key) in enumerate(stimuli_data):
    x, y = positions[i]
    stimuli.append({
        'freq': freq, 'text': text, 'key': key,
        'rect': pygame.Rect(x, y, square_size, square_size),
        'period_frames': FPS / freq if freq != 0 else float('inf'),
    })

# --- 4. 主循环与状态管理 ---
clock = pygame.time.Clock()
frame_counter = 0
feedback_message = ""
feedback_timer = 0
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                running = False
            for stimulus in stimuli:
                if event.key == stimulus['key']:
                    feedback_message = f"识别控制指令：{stimulus['text']}"
                    feedback_timer = int(FPS * 1.5)
                    print(f"检测到按键: {stimulus['text']}")
                    break
    screen.fill(BLACK)
    if feedback_timer > 0:
        # --- 最终修改点: 方块和文字都用白色 ---
        for stimulus in stimuli:
            pygame.draw.rect(screen, WHITE, stimulus['rect']) # 1. 方块背景用白色
            
            # 2. 方块内的文字也用白色，使其“消失”在背景中
            text_surface = font_stimulus.render(stimulus['text'], True, WHITE) 
            text_rect = text_surface.get_rect(center=stimulus['rect'].center)
            screen.blit(text_surface, text_rect)
            
        # 屏幕中央显示红色反馈文字
        feedback_surface = font_feedback.render(feedback_message, True, RED)
        feedback_rect = feedback_surface.get_rect(center=(WIDTH / 2, HEIGHT / 2))
        screen.blit(feedback_surface, feedback_rect)
        feedback_timer -= 1
    else:
        # 正常闪烁状态
        for stimulus in stimuli:
            half_period = stimulus['period_frames'] / 2
            is_on = (frame_counter % stimulus['period_frames']) < half_period if half_period > 0 else True
            color = WHITE if is_on else BLACK
            pygame.draw.rect(screen, color, stimulus['rect'])

            # 正常状态下文字为白色
            text_surface = font_stimulus.render(stimulus['text'], True, WHITE)
            text_rect = text_surface.get_rect(center=stimulus['rect'].center)
            screen.blit(text_surface, text_rect)
            
    pygame.display.flip()
    frame_counter += 1
    clock.tick(FPS)

# --- 清理 ---
pygame.quit()
sys.exit()
