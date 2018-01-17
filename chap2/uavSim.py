import matplotlib.pyplot as plt
import numpy as np
import sys
sys.path.append('..')  # add parent directory
import uavParam as P
# from signalGenerator import signalGenerator
from uavAnimation import uavAnimation
# from plotData import plotData


import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
# import matplotlib.animation as animation

from matplotlib.widgets import Slider


# instantiate reference input classes
# reference = signalGenerator(amplitude=0.5, frequency=0.1)
# zRef = signalGenerator(amplitude=0.9, frequency=0.7, y_offset=2.5) 
# hRef = signalGenerator(amplitude=0.7, frequency=0.4, y_offset=2.5) 
# ztRef = signalGenerator(amplitude=0.6, frequency=0.6, y_offset=2.5) 
# thetaRef = signalGenerator(amplitude=np.pi/2, frequency=0.1)
# fRef = signalGenerator(amplitude=5, frequency=.5)

# print zRef

# instantiate the simulation plots and animation
# dataPlot = plotData()


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

yaw0 = 0
pitch0 = 0
roll0 = 0
x0 = 0
y0 = 0
z0 = 0

# Define an axes area and draw a slider in it
yaw_slider_ax  = fig.add_axes([0.15, 0.3, 0.7, 0.03], axisbg='white')
yaw_slider = Slider(yaw_slider_ax, 'Yaw', -180.0, 180.0, valinit=yaw0)

# Draw another slider
pitch_slider_ax = fig.add_axes([0.15, 0.25, 0.7, 0.03], axisbg='white')
pitch_slider = Slider(pitch_slider_ax, 'Pitch', -180, 180.0, valinit=pitch0)

# Draw another slider
roll_slider_ax = fig.add_axes([0.15, 0.2, 0.7, 0.03], axisbg='white')
roll_slider = Slider(roll_slider_ax, 'Roll', -180.0, 180.0, valinit=roll0)

# Draw another slider
x_slider_ax = fig.add_axes([0.15, 0.15, 0.7, 0.03], axisbg='white')
x_slider = Slider(x_slider_ax, 'X', -1.0, 1.0, valinit=x0)

y_slider_ax = fig.add_axes([0.15, 0.1, 0.7, 0.03], axisbg='white')
y_slider = Slider(y_slider_ax, 'Y', -1.0, 1.0, valinit=y0)

z_slider_ax = fig.add_axes([0.15, 0.05, 0.7, 0.03], axisbg='white')
z_slider = Slider(z_slider_ax, 'Z', -1.0, 1.0, valinit=z0)


animation = uavAnimation(fig)

def sliders_on_changed(val):
    animation.drawUAV([yaw_slider.val, pitch_slider.val, roll_slider.val, 
                x_slider.val, y_slider.val, z_slider.val])
    fig.canvas.draw_idle()


yaw_slider.on_changed(sliders_on_changed)
pitch_slider.on_changed(sliders_on_changed)
roll_slider.on_changed(sliders_on_changed)
x_slider.on_changed(sliders_on_changed)
y_slider.on_changed(sliders_on_changed)
z_slider.on_changed(sliders_on_changed)


plt.show()