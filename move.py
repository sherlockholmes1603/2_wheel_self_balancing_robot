
import sys
#import pidcontrol as pid
import numpy as np
import pybullet as p
import math
import time
import pybullet_data
import controlpy
import time


# # LQR Controller

# Here, we are working on LQR controller. We have used control library [[1](#PythonControl)] of python to work on lqr. 
# For the lqr controllers, the state-space equations of the system are required. 
# The above equation is derived from [[2](#SystemEquation)] .
# Here,
#(M)       mass of the cart                         2 kg
#(m)       mass of the pendulum                     5 kg
#(b)       coefficient of friction for cart         0.5 N/m/sec
#(l)       length to pendulum center of mass        0.1 m
#(I)       mass moment of inertia of the pendulum   0.10500391309238813 kg.m^2
#(F)       force applied to the cart
#(x)       cart position coordinate
#(theta)   pendulum angle from vertical (down)

#We can calculate
#x(t+1)=A*x(t)+B*u
#x(t+1)=k*x(t)

M = 2
m = 5
b = 0.5
I = 0.10500391309238813
g = 9.8
l = 0.1

####################### write your code here #####################################################################################
#Refer to "test_lqr.py" for example code
#Variable : "A" : 2D numpy array 
#Variable : "A" : 2D numpy array
#write here
A = np.matrix(  [[0, 1, 0, 0], [0, -(m+M)*g*b/M, g*m/M, 0],[0, 0, 0, 1],[0, 0, (M+m)*g/(l*M), 0]])

B = np.matrix(	[[0],[1/M],[0],[1/(l*M)]])

###################################################################################################################################

# The class for LQR

Q = np.array([[ 100,   0],[  0, 1000]])
R = 0.0001
# K,S,e = lqr(A,B,Q,R)
# print(K)
# print(S)
# print(e)

# From [[2](#SystemEquation)] , we get the cost function 
# J = int (x^T Q x +u^T R u)dt
# Here, we need to tune Q matrix and R such that the value of "J" is minimum. The lqp function automatically does that for you

class SelfBalanceLQR:
    '''
    Purpose:
    ---
    Class Describing the gains of LQR and various functions
    Functions:
    ---
        __init__ : Called by default to initilize the variables in LQR
        callback : It takes the data from sensors/bots and accordingly predicts the next state and respecive action
        callback_q : It can be used to change the value of gains during execution
        callback_r : It can be used to change the value of gains during execution
    Example initialization and function call:
    ---
    balance=SelfBalanceLQR()
    vel=balance.callback(data)
    '''
    def __init__(self):
        self.xvelMin=-.01# x velocity
        self.xvelMax =0
        self.yMin = -0.01#yaw
        self.yMax = 0
        self.y_ = 0
        self.Q = np.array([[ 100, 0, 0, 0],[0,100,0,0],[0,0,100,0],[0,0,0, 1000]])
        self.R = [[10]]
        self.K,self.S,self.e = controlpy.synthesis.controller_lqr(A,B,self.Q,self.R)
    def callback(self,np_x):
        ######################################## write yor code here ######################################################
        # calculate the state using data calulated from bot
        # Variable np_x : 2D array of state to be multiplie with A 
        # Variable y : integer representing the state value that is used to calculate error/other states 
        #write here
        y = np_x[2]
        u_t=-np.matmul(self.K,np_x) 

        #print(np_x,self.K,u_t)
        #print(np_x.shape,self.K.shape,u_t.shape,A.shape,B.shape)
        x = (np.matmul(A,np_x)+np.matmul(B,u_t)) ##### might have to change this based on your state equation
        xvel = x[1]
        #print(xvel)

        ####################################################################################################################
        
        #storing previous values for evaluation 
        if y>self.yMax:
            self.yMax = y
        elif y<self.yMin:
            self.yMin =y
        if xvel>self.xvelMax:
            self.xvelMax=xvel
        elif xvel<self.xvelMin:
            self.xvelMin = xvel
            
        linear_vel = [xvel,0,0]
        angular_vel=[0,0,0]
        #print("Max vel " + str(self.xvelMax) + " & Min vel " + str(self.xvelMin) + " Max y " + str(self.yMax*180/3.1416) +" & Min y" + str(self.yMin*180/3.1416))
        #print("Velocity "+ str(xvel)+ " & yaw " + str(y))
        self.y_ = y # storing th previous value of state 
        return xvel-np_x[1] # returning the difference of current state and next state to compensate error
        
    def callback_q(self,data):
        q = data.data
        self.Q = np.array([[ q,   0],[  0, 10*q]])
        self.K,self.S,self.e = controlpy.synthesis.controller_lqr(A,B,self.Q,self.R)
        
    def callback_r(self,data):
        r = data.data
        self.R = r
        self.K,self.S,self.e = controlpy.synthesis.controller_lqr(A,B,self.Q,self.R)


def synthesizeData(robot,dt,target_pitch):
    '''
    Purpose:
    ---
    Calculate the current state(position , velocity , orienation etc.)
    Input Arguments:
    ---
    `robot` :  integer
        object id of bot spawned in pybullet

    Returns:
    ---
    `data` :  1D array 
        list of information required for calculation
    Example call:
    ---
    data=synthesizeData(robot)
    '''
    # print("----------------------------------------------------------------------------------------------------------------")
    # print("Dynamic Info of Base : ",p.getDynamicsInfo(robot, -1),end="\n")
    #0->mass , 3->local inertial pos
    posi_orie = p.getBasePositionAndOrientation(robot)
    coordinate = posi_orie[0]
    orient = p.getEulerFromQuaternion(posi_orie[1])
    # print("Base position and Orientation : " , p.getBasePositionAndOrientation(robot),end="\n")
    #1->orientation
    link = p.getLinkState(robot, 1, computeLinkVelocity=1)

    com = p.getDynamicsInfo(robot, -1)[2][2]
    com += coordinate[2] 
    # print("Centre of mass - ", com)
    x = coordinate[0]
    link_orie = p.getEulerFromQuaternion(link[3])
    pitch = orient[1]
    # print("pitch: ",pitch)
    velocity = p.getBaseVelocity(robot)
    x_dot = link[6][0]
    pitch_dot = velocity[1][1]
    # print(x_dot)
    # print((pitch_dot))
    #information required yaw
    #imu sensor , kp ,ki ,kd
    #set cmd_vel 
    ################################################## write our code here ######################################################
    #Varible : data ,1D array that contains all the state variable required to implement LQR
    # For hints refer to statements above and choose an apt state variable
    #write here
    #############################################################################################################################
    data = np.matrix([[x],[x_dot],[pitch - target_pitch],[pitch_dot]])

    return data



prev_yaw_error = 0
def yaw_pid(target_Yaw):
    kp = 10
    kd = 100
    orient = p.getBasePositionAndOrientation(robot)[1]
    yaw = p.getEulerFromQuaternion(orient)[2]
    error = yaw -target_Yaw
    yaw_vel = p.getBaseVelocity(robot)[1][2]
    trq_ = kp*error + kd*yaw_vel
    trq_ /= 1000
    trq_ = max(-0.005,min(0.005,trq_))
    print("trq_: ",trq_)
    print("error: ",error)
    prev_yaw_error = error
    return trq_

if __name__ == "__main__":
    '''
    Purpose:
    ---
        Setup the pybullet environment and calculation of state variables and respective action to balance the bot
    '''
    # connecting to pybullet
    id = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    plane = p.loadURDF("plane.urdf")
    p.setGravity(0, 0, -9.8)
    robot = p.loadURDF("urdf/self_balance.urdf" , [0,0,0.2])
    num = p.getNumJoints(robot)
    # for i in range(num):
    #     info = p.getJointInfo(robot, i)
    #     print(info,end="\n")
    #     link_name = info[12].decode("ascii")
    #     if link_name == "left_wheel": left_wheel = j
    #     if link_name == "right_wheel": wheel_foot = j
    left_joint=0
    right_joint=1
    maxForce = 0
    mode = p.VELOCITY_CONTROL
    p.setJointMotorControl2(robot, left_joint,controlMode=mode, force=maxForce)
    p.setJointMotorControl2(robot, right_joint,controlMode=mode, force=maxForce)


    # p.changeDynamics(robot,left_joint,lateralFriction=0.7,spinningFriction=0.5,rollingFriction=0.5)
    # p.changeDynamics(robot,right_joint,lateralFriction=0.7,spinningFriction=0.5,rollingFriction=0.5)
    balance=SelfBalanceLQR()
    dt = 0.01
    init_time = time.time()
    c = 0
    while(True):
        dt = time.time() - init_time
        data=synthesizeData(robot,dt,0)
        keys = p.getKeyboardEvents()
        trq_ = 0
        for k,v in keys.items():
            if (k == p.B3G_UP_ARROW and (v & p.KEY_IS_DOWN)):
                data = synthesizeData(robot, dt, 0.2)
                # sleep(0.1)
            if (k == p.B3G_DOWN_ARROW and (v & p.KEY_IS_DOWN)):
                data = synthesizeData(robot, dt, -0.2)
                # sleep(0.1)
            if (k == p.B3G_LEFT_ARROW and (v & p.KEY_IS_DOWN)):
                trq_ = 0.01
                # time.sleep(0.1)
            if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_IS_DOWN)):
                trq_ = -0.01
                # time.sleep(0.1)
        # data=synthesizeData(robot,dt)#get data from simulation and bot
        trq=balance.callback(data)#call to function to implement algorithm
        # yaw_data = synthesizeYawData(robot, dt, c)
        # trq_ = yaw_pid(c)
        trq = max(-2,min(2,trq))
        print("trq: ",trq)
        
        p.setJointMotorControl2(robot, left_joint , p.TORQUE_CONTROL, force = -trq-trq_)
        p.setJointMotorControl2(robot, right_joint , p.TORQUE_CONTROL, force = trq-trq_)
        p.stepSimulation()
        time.sleep(0.01)
        init_time = time.time()



p.disconnect()
# # Reference
# [1] Python Control Package , [link](https://github.com/markwmuller/controlpy)
# [2] Control System Tutorials , [link](http://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=SystemModeling)
# https://automaticaddison.com/linear-quadratic-regulator-lqr-with-python-code-example/
