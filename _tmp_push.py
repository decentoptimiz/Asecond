# -*- coding: utf-8 -*-
import subprocess

GIT = r"C:\Program Files\Git\cmd\git.exe"
REPO = r"D:\Asecond"

MSG = """V1.2 双通道示波器：移除 LCD、曲线平滑、指标补齐、前端偏置标定

1. 移除 LCD / 触摸驱动（FSMC / fsmc.c），仅保留串口屏 HMI，Flash 46.5KB -> 23.7KB
2. 串口屏：单曲线控件双通道，DRAW_POINTS 240 -> 400（50 点/div，满足水平分辨率要求）
3. hmi_text.h：GBK 字节转义的中文参数文本（频率/峰峰值/平均值/相位差/波形类型等）
4. 波形重采样：上采样线性插值、下采样区间平均，消除阶梯感与混叠
5. 波形识别改用峰均比法（方波 1.0 / 正弦 1.414 / 三角 1.732，阈值 1.20 / 1.58）
6. 新增 AUTOSCALE：两通道同时选档 + 触发中点 + 时基自适应
7. 前端接口：ADC 换算扣除 2.5V 直流偏置（ADC_BIAS_V），新增双通道增益标定表
   gain_cal[2][3] = {40,4,1}，配合 CD4051 三档（Rf = 390k / 30k / 0 欧，Rg = 10k）
8. 修复 USART1 接收中断未使能（NVIC + USART1_IRQHandler + 溢出恢复），触摸按钮恢复响应
9. 档位文本两通道同档时合并显示，避免控件宽度截断

构建：Keil MDK-ARM 02LCD_F4，0 error / 7 warning，Flash 23732 B，RAM 101288 B

Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>
"""

def run(*args, check=True):
    p = subprocess.run([GIT, "--no-pager"] + list(args), cwd=REPO, capture_output=True)
    out = p.stdout.decode("utf-8", "replace").strip()
    err = p.stderr.decode("utf-8", "replace").strip()
    print("$ git " + " ".join(args) + "  (rc=%d)" % p.returncode)
    if out:
        print(out)
    if err:
        print("[stderr]", err)
    if check and p.returncode != 0:
        raise SystemExit("FAILED: git " + " ".join(args))
    return p.returncode

run("add", "-A")
run("rm", "--cached", "-q", "_tmp_git.py", check=False)
run("status", "--short")
run("commit", "-m", MSG)
run("log", "--oneline", "-3")
run("push", "origin", "main")
run("log", "--oneline", "-2")
