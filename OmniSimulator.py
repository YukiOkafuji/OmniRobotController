import numpy as np
import math as mt
import matplotlib.pyplot as plt
import matplotlib.animation as ani

# set parameters
dt = 0.01

# set target position
TargetX = -3.0
TargetY = 2.0
TargetTheta = -1.0

# set gains
k1 = 5.0
k2 = 5.0
k3 = 3.0

class OmniRobotState():
    def __init__(self):
        # Robot Parameter
        self.L = 0.15 # need to measure

        # Robot State
        self.STATES = np.zeros((3, 1))       # X, Y, Theta
        self.INPUT_GLOBAL = np.zeros((3, 1)) # Vx, Vy, Omega
        self.INPUT_LOCAL = np.zeros((3, 1))  # v1, v2, v3
        self.ERROR = np.zeros((3, 1))        # e1, e2, e3

        # Convert velocity
        self.CONVERT_L2G = np.asarray([[-0.5, -0.5, 1.0], [mt.sqrt(3)*0.5, -mt.sqrt(3)*0.5, 0], [1/self.L, 1/self.L, 1/self.L]]) # from INPUT_LOCAL to INPUT_GLOBAL
        self.CONVERT_G2L = np.linalg.inv(self.CONVERT_L2G) # from INPUT_GLOBAL to INPUT_LOCAL

        # Target Position
        self.TARGET_POSITION = np.asarray([[TargetX], [TargetY], [TargetTheta]])

        # Gains
        self.GAINS = np.asarray([[k1], [k2], [k3]])

    def Controller(self):
        self.ERROR = self.TARGET_POSITION - self.STATES                 # Calculate Errors
        self.INPUT_GLOBAL = self.GAINS*self.ERROR                       # Calculate Control Input in Global
        self.INPUT_LOCAL = np.dot(self.CONVERT_G2L, self.INPUT_GLOBAL)  # Convert Control Input to Local
        self.WheelInput(self.INPUT_LOCAL)                               # Send Control Input to Robot

        return self.STATES

    def WheelInput(self, v):
        self.INPUT_LOCAL = v
        self.INPUT_GLOBAL = np.dot(self.CONVERT_L2G, self.INPUT_LOCAL) # Convert Control Input to Global
        self.STATES += self.INPUT_GLOBAL*dt                            # Update Robot State

        return self.STATES

    
def update_anim(i):
    ### FF Controller ###
    #ConstInput = np.asarray([[1.0], [-1.0], [0.0]])  # move to y direction
    #ConstInput = np.asarray([[-1.0], [-1.0], [2.0]]) # move to x direction
    #ConstInput = np.asarray([[1.0], [1.0], [1.0]])   # rotatioin here
    #states = RS.WheelInput(ConstInput) # move to x direction
    
    ### FB Controller ###
    states = RS.Controller()

    # reset plot state
    plt.cla()
    ax = fig.add_subplot(111)
    plt.title("Omni Wheel Robot Simulation")
    plt.ylim(-5, 5)
    plt.xlim(-5, 5)
    plt.grid()

    # draw target robot
    circle2 = plt.Circle((TargetX, TargetY), 0.3, fc = "white", ec = "black")
    ax.add_patch(circle2)
    arrow2 = plt.arrow(x=TargetX, y=TargetY, dx=mt.sin(-TargetTheta), dy=mt.cos(-TargetTheta), width=0.01, color='b')
    ax.add_patch(arrow2)

    # draw robot state
    circle = plt.Circle((states[0][0], states[1][0]), 0.3, fc = "grey", ec = "black")
    ax.add_patch(circle)
    arrow = plt.arrow(x=states[0][0], y=states[1][0], dx=mt.sin(-states[2][0]), dy=mt.cos(-states[2]), width=0.01, color='r')
    ax.add_patch(arrow)


# set robot state class
RS = OmniRobotState()

# set figure
fig = plt.figure()

# start animation
animation = ani.FuncAnimation(fig, update_anim, interval=100, frames=50)
plt.show()
