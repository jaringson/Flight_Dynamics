import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d.axes3d import Axes3D
import numpy as np 
import uavParam as P


class uavAnimation:
    '''
        Create pendulum animation
    '''
    def __init__(self, fig):
        self.fig = fig
        self.ax = self.fig.add_subplot(111, projection='3d')

        

        # Adjust the subplots region to leave some space for the sliders and buttons
        self.fig.subplots_adjust(bottom=0.5)

        self.plane = np.array([[0,0,0],
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

        # print self.plane
        # print self.plane.shape
        self.line = []
        
        [self.line] = self.ax.plot(*np.array([[1,0,0],[0,1,0],[0,0,1]]).dot(self.plane), linewidth=2, color='red')

        self.ax.set_xlim([-1, 1])
        self.ax.set_ylim([-1, 1])
        self.ax.set_zlim([-1, 1])
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.invert_zaxis()
        self.ax.invert_yaxis()

        # plt.show()

        # Draw pendulum is the main function that will call the functions:
        # drawCart, drawCircle, and drawRod to create the animation.
    def drawUAV(self, val):

        yaw = -np.radians(val[0])
        pitch = -np.radians(val[1])
        roll = -np.radians(val[2])

        T = np.array([[1,0,0,val[3]],
                    [0,1,0,val[4]],
                    [0,0,1,val[5]],
                    [0,0,0,1]])

        
        
        Rv_v1 = np.array([[np.cos(yaw),np.sin(yaw),0,0],
                      [-np.sin(yaw),np.cos(yaw),0,0],
                      [0,0,1,0],
                      [0,0,0,1]])
        Rv1_v2 = np.array([[np.cos(pitch),0,-np.sin(pitch),0],
                      [0,1,0,0],
                      [np.sin(pitch),0,np.cos(pitch),0],
                      [0,0,0,1]])
        Rv2_b = np.array([[1,0,0,0],
                      [0,np.cos(roll),np.sin(roll),0],
                      [0,-np.sin(roll),np.cos(roll),0],
                      [0,0,0,1]])


        T_all = T.dot(Rv2_b).dot(Rv1_v2).dot(Rv_v1)
        print T_all
        R = T_all[0:3,0:3]
        T = T_all[0:3,3]
        print T
        # print self.plane
        temp_plane = np.array([self.plane[0] + T[0],
                        self.plane[1] + T[1],
                        self.plane[2] + T[2] ])
        print self.plane
        print temp_plane

        self.line.remove()
        [self.line] = self.ax.plot(*R.dot(temp_plane), linewidth=2, color='red')

        self.fig.canvas.draw_idle()




# Used see the animation from the command line
if __name__ == "__main__":

    fig = plt.figure()

    simAnimation = uavAnimation(fig)    # Create Animate object
    z = 0.0                               # Position of cart, m
    theta = 0.0*np.pi/180                 # Angle of pendulum, rads
    simAnimation.drawUAV([1, 2, 3, 4, 5, 6])  # Draw the pendulum
    #plt.show()
    # Keeps the program from closing until the user presses a button.
    print('Press key to close')
    plt.waitforbuttonpress()
    plt.close()