# -*- coding: utf-8 -*-
"""
register = np.array([0, 1, 1, 0]) seed 2
register = np.array([0, 0, 1, 1]) seed 3
register = np.array([1, 1, 0, 1]) seed 10
"""

import numpy as np
import matplotlib.pyplot as plt


b = 0.2
sample_rate = 5


def generate_mls(n, taps, seed=None):
    # 初始化寄存器(零向量除外)
    np.random.seed(seed)  # 固定随机种子
    register = np.random.randint(2, size=n)
    print(register)
    if np.all(register == 0):
        print("Reset the seed")
    
    # 存储生成的MLS
    mls_sequence = []
    
    for _ in range(2**n - 1):
        # 计算反馈值
        feedback = np.mod(np.sum(register[taps]), 2)
        
        # 更新寄存器
        register = np.roll(register, 1)
        register[0] = feedback
        
        # 添加到MLS序列
        mls_sequence.append(register[-1])
    
    return mls_sequence

# 定义MLS的阶数和反馈位
n = 4
taps = [2, 3]

# 生成MLS序列
binary_mls = generate_mls(n, taps, 2)

# 打印生成的MLS序列
print(binary_mls)

mls = np.where(np.array(binary_mls) == 1, b, -b)
#t = np.arange(0, len(mls) * dt, dt)
t = 1.0 / sample_rate * np.arange(2**4-1)

plt.step(t, mls, where='post', color='b')
plt.plot([t[-1], t[-1]+1/sample_rate], [mls[-1], mls[-1]], color='b')
plt.xlabel('Time')
plt.ylabel('Amplitude')
plt.title('Rectangular Waveform from PRBS')
plt.show()

