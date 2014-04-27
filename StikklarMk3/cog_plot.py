import matplotlib.pyplot as plt
import math
from matplotlib import ticker

tranTime = 98
stepsInCycle = 16
cycleTime = (stepsInCycle*tranTime)/1000.0
f = 1.0 #/ cycleTime
a_lr = 0.5
a_fb = 0.1

A_lrx = 0.0
A_lry = -170.0
A_fbx = 170.0
A_fby = 0.0
w = math.pi*2*f

def cog(t):
    cycleOffset = 0#-math.pi/2
    m_lr = math.sin((w*t)+math.pi/2) * a_lr
    m_fb = math.sin(2*w*t) * a_fb

    cogx = (A_lrx*m_lr) + (A_fbx*m_fb)
    cogy = (A_lry*m_lr) + (A_fby*m_fb)
    return -cogx, -cogy


time = []
xx = []
yy = []

for i in xrange(stepsInCycle+1):
    t = float(i)/stepsInCycle
    x, y = cog(t)
    time.append(t)
    xx.append(x)
    yy.append(y)

# Create plots with pre-defined labels.
# Alternatively, you can pass labels explicitly when calling `legend`.
fig, ax = plt.subplots()
ax.plot(time, xx, 'k--', label='X axis front/back')
ax.plot(time, yy, 'k:', label='Y axis left/right')

ax.xaxis.set_major_locator(ticker.MultipleLocator(base=0.25))
ax.grid(True)
#ax.plot(a, c+d, 'k', label='Total message length')

# Now add the legend with some customizations.
legend = ax.legend(loc='upper center', shadow=True)

# The frame is matplotlib.patches.Rectangle instance surrounding the legend.
frame  = legend.get_frame()
frame.set_facecolor('0.90')

# Set the fontsize
for label in legend.get_texts():
    label.set_fontsize('large')

for label in legend.get_lines():
    label.set_linewidth(1.5)  # the legend line width
plt.show()