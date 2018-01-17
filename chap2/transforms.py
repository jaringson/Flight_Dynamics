
from numpy import pi, sin
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
# import matplotlib.animation as animation

from matplotlib.widgets import Slider

def draw_plane_nwu(plane_in):
    R = np.array([[1,0,0],
                  [0,-1,0],
                  [0,0,-1]])
    p = R.dot(plane_in)
    return p[0,:], p[1,:], p[2,:]

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Adjust the subplots region to leave some space for the sliders and buttons
fig.subplots_adjust(bottom=0.25)

t = np.arange(0.0, 1.0, 0.001)
azim_0 = 0
ele_0 = 0
tilt_0 = 0

# plane in ned
plane = np.array([[0,0,0],
                  [0.5,0,0],
                  [0.1,0,0],
                  [0,0.5,-0.1], #left wing
                  [0.1,0,0],
                  [0,-0.5,-0.1], #right wing
                  [0.1,0,0],
                  [-0.5,0,0],
                  [-0.5,0,-0.25],
                  [-0.5,0.1,-0.25],
                  [-0.5,-0.1,-0.25]]).T
# Draw the initial plot
# The 'line' variable is used for modifying the line later
[line] = ax.plot(*draw_plane_nwu(np.array([[-1,0,0],[0,-1,0],[0,0,1]]).dot(plane)), linewidth=2, color='red')
# [line2] = ax.plot(*draw_plane_nwu(np.array([[-1,0,0],[0,-1,0],[0,0,1]]).dot(plane)), linewidth=2, color='blue')
ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Define an axes area and draw a slider in it
azim_slider_ax  = fig.add_axes([0.15, 0.15, 0.7, 0.03], axisbg='white')
azim_slider = Slider(azim_slider_ax, 'Azim', -180.0, 180.0, valinit=azim_0)

# Draw another slider
ele_slider_ax = fig.add_axes([0.325, 0.1, 0.35, 0.03], axisbg='white')
ele_slider = Slider(ele_slider_ax, 'Elev', -90.0, 90.0, valinit=ele_0)

# Draw another slider
tilt_slider_ax = fig.add_axes([0.15, 0.05, 0.7, 0.03], axisbg='white')
tilt_slider = Slider(tilt_slider_ax, 'Tilt', -180.0, 180.0, valinit=tilt_0)

# Define an action for modifying the line when any slider's value changes
def sliders_on_changed(val):
    azim = np.radians(azim_slider.val)
    ele = np.radians(ele_slider.val)
    tilt = np.radians(tilt_slider.val)
    Rt = np.array([[1,0,0],
                  [0,np.cos(-tilt),-np.sin(-tilt)],
                  [0,np.sin(-tilt),np.cos(-tilt)]])
    Re = np.array([[np.cos(-ele),0,np.sin(-ele)],
                  [0,1,0],
                  [-np.sin(-ele),0,np.cos(-ele)]])
    Ra = np.array([[np.cos(azim),-np.sin(azim),0],
                  [np.sin(azim),np.cos(azim),0],
                  [0,0,1]])
    Rrev = np.array([[-1,0,0], # rotates the plane to face you instead of away from you
                  [0,-1,0],
                  [0,0,1]])

    R = Rrev.dot(Rt).dot(Re).dot(Ra)

    global line
    line.remove()
    [line] = ax.plot(*draw_plane_nwu(R.dot(plane)), linewidth=2, color='red')

    # now assuming Z_1*Y_2*X_3
    pitch = -np.arcsin(R[2,0])
    roll = np.arctan2(R[2,1],R[2,2])
    yaw = np.arctan2(R[1,0],R[0,0])
    print 'yaw', np.degrees(yaw), 'pitch', np.degrees(pitch), 'roll',np.degrees(roll)

    # Rr = np.array([[1,0,0],
    #               [0,np.cos(roll),-np.sin(roll)],
    #               [0,np.sin(roll),np.cos(roll)]])
    # Rp = np.array([[np.cos(pitch),0,np.sin(pitch)],
    #               [0,1,0],
    #               [-np.sin(pitch),0,np.cos(pitch)]])
    # Ry = np.array([[np.cos(yaw),-np.sin(yaw),0],
    #               [np.sin(yaw),np.cos(yaw),0],
    #               [0,0,1]])
    # R = Ry.dot(Rp).dot(Rr)
    # global line2
    # line2.remove()
    # [line2] = ax.plot(*draw_plane_nwu(R.dot(plane)), linewidth=2, color='blue')

    fig.canvas.draw_idle()

azim_slider.on_changed(sliders_on_changed)
ele_slider.on_changed(sliders_on_changed)
tilt_slider.on_changed(sliders_on_changed)

plt.show()
