import pybullet as p
import time
import pybullet_data
import numpy as np
import math


## setup
useMaximalCoordinates = False
p.connect(p.GUI)
p.setGravity(0, 0, -10)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

pole = p.loadURDF("./data/2link_robot.urdf", [0, 0, 0], useMaximalCoordinates=useMaximalCoordinates, useFixedBase=True)
p.resetBasePositionAndOrientation(pole, [0, 0, 0], [0, 0, 0, 1])
numJoints = p.getNumJoints(pole)

p.setJointMotorControl2(pole, 0, p.POSITION_CONTROL, targetPosition=0, force=0)
p.setJointMotorControl2(pole, 1, p.POSITION_CONTROL, targetPosition=0, force=0)
timeStepId = p.addUserDebugParameter("timeStep", 0.001, 0.1, 0.01)
desiredXId = p.addUserDebugParameter("desired_X",-5,5,2)
desiredYId = p.addUserDebugParameter("desired_Y",-5,5,1)
useRealTimeSim = False
p.setRealTimeSimulation(useRealTimeSim)

jointIds=[]
paramIds=[]

for j in range (p.getNumJoints(pole)):
	jointIds.append(j);

numJoints = p.getNumJoints(pole)




prev = [0, 0, 0]

kp = 100
kd = 10

jointAngle = [0, 0, 0]
theta = [0, 0, 0]
l0 = 2
l1 = 1


while p.isConnected():
    p.stepSimulation()
    timeStep = p.readUserDebugParameter(timeStepId)
    p.setTimeStep(timeStep)
    time.sleep(timeStep)
    x = p.readUserDebugParameter(desiredXId)
    y = p.readUserDebugParameter(desiredYId)

    # IK
    # with desired_q coordinate, calculate IK and figure out the joint position
    # how to call the length of each joint?

    for i in range(len(jointIds)):
        jointState = p.getJointState(pole,i)
        jointAngle[i] = jointState[0] # save the current joint position

    
    sign = 1
    if(jointAngle[1]<0): sign = -1 # 기존 theta1의 sign 따라서 theta1 설정
    tmp = (x*x+y*y-l0*l0-l1*l1)/(2.0*l0*l1)
    if(abs(tmp)<=1):
        theta[1] = sign * math.acos(tmp)
        theta[0] = math.atan(y/x)-math.atan((l1*math.sin(theta[1]))/(l0+l1*math.cos(theta[1])))
    else: 
        theta[1] = jointAngle[1]
        theta[0] = jointAngle[0]
    

    # PD
    for i in range(len(jointIds)):
        qError = jointAngle[i]-theta[i]
        qDot = (qError-prev[i])/timeStep
        
        torques = -kp*qError-kd*qDot
        targetPos = theta[i]
        
        p.setJointMotorControl2(pole,jointIds[i],p.TORQUE_CONTROL,force = torques)
        # p.setJointMotorControl2(pole,jointIds[i],p.POSITION_CONTROL,desired_q[i])
        prev[i] = qError
