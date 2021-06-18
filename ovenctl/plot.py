#!/usr/bin/env python

import pylab as plt
import matplotlib
import time
import sys

matplotlib.use("TkAgg")

plt.ioff()

fig, ax = plt.subplots(1)

xs = []
y1s = []
y2s = []

x = 0
for line in sys.stdin:
  y1, y2 = line.split(' ')
  xs.append(x)
  y1s.append(float(y1))
  y2s.append(float(y2))

  ax.clear()
  ax.plot(xs, y1s)
  ax.plot(xs, y2s)
  x = x + 0.5
  plt.draw()
  plt.pause(0.001)

plt.ioff()
plt.show()
