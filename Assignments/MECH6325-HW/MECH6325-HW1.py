# -*- coding: utf-8 -*-
"""
Created on Fri Sep 11 07:17:41 2020

@author: Jonas
"""

import numpy as np
import matplotlib.pyplot as plt


# Problem 16


k = 2
n = 10000
X_1 = np.zeros(n)


for i, x in enumerate(X_1):
    j = 0
    temp = 0
    while j < k:
        temp += np.random.uniform(-0.5, 0.5)
        j += 1
    X_1[i] = temp / k


k = 4
n = 10000
X_2 = np.zeros(n)

for i, x in enumerate(X_2):
    j = 0
    temp = 0
    while j < k:
        temp += np.random.uniform(-0.5, 0.5)
        j += 1
    X_2[i] = temp / k



fig, (ax1, ax2) = plt.subplots(2, 1)

x_min = -0.5
x_max = 0.5

ax1.set_xlim(x_min, x_max)
ax2.set_xlim(x_min, x_max)

ax1.hist(X_1, 50, range = (x_min, x_max))
ax2.hist(X_2, 50, range = (x_min, x_max))
