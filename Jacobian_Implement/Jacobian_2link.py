from os import link
import pybullet as p
import time
import pybullet_data
import numpy as np
import math



def getJointStates(robot):
  joint_states = p.getJointStates(robot, range(p.getNumJoints(robot)))
  joint_positions = [state[0] for state in joint_states]
  joint_velocities = [state[1] for state in joint_states]
  joint_torques = [state[3] for state in joint_states]
  return joint_positions, joint_velocities, joint_torques


def getMotorJointStates(robot):
  joint_states = p.getJointStates(robot, range(p.getNumJoints(robot)))
  joint_infos = [p.getJointInfo(robot, i) for i in range(p.getNumJoints(robot))]
  joint_states = [j for j, i in zip(joint_states, joint_infos) if i[3] > -1]
  joint_positions = [state[0] for state in joint_states]
  joint_velocities = [state[1] for state in joint_states]
  joint_torques = [state[3] for state in joint_states]
  return joint_positions, joint_velocities, joint_torques



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
desiredXId = p.addUserDebugParameter("desired_X",-3,3,2)
desiredYId = p.addUserDebugParameter("desired_Y",-3,3,1)
useRealTimeSim = False
p.setRealTimeSimulation(useRealTimeSim)

jointIds=[]

numJoints = p.getNumJoints(pole)
numLinks = numJoints

for j in range (numJoints):
	jointIds.append(j);







prev = [0, 0, 0]

kp = 10
kd = 0.0001
ki = 0.01

jointAngle = [0, 0, 0]
jointVel = [0, 0, 0]
jointLength = []
linkPos = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
theta = [0, 0, 0]
l0 = 2
l1 = 1

def RotZ(angle, trans):
    arr = np.array([[math.cos(angle), -math.sin(angle), 0, trans], [math.sin(angle), math.cos(angle), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    return arr


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

    for i in range(numLinks):
        linkState = p.getLinkState(pole,i)
        linkPos[i][0] = linkState[0][0] # save the current link position
        linkPos[i][1] = linkState[0][1]
        linkPos[i][2] = linkState[0][2]

    for i in range(len(jointIds)-1):
        jointState = p.getJointState(pole,i)
        jointAngle[i] = jointState[0] # save the current joint position
        jointVel[i] = jointState[1]

        lx = linkPos[i][0]-linkPos[i+1][0]
        ly = linkPos[i][1]-linkPos[i+1][1]
        lz = linkPos[i][2]-linkPos[i+1][2]
        dist = math.sqrt(lx*lx+ly*ly+lz*lz)
        jointLength.append(dist)

    
    # traditional method
    # sign = 1
    # if(jointAngle[1]<0): sign = -1 # 기존 theta1의 sign 따라서 theta1 설정
    # tmp = (x*x+y*y-l0*l0-l1*l1)/(2.0*l0*l1)
    # if(abs(tmp)<=1):
    #     theta[1] = sign * math.acos(tmp)
    #     theta[0] = math.atan(y/x)-math.atan((l1*math.sin(theta[1]))/(l0+l1*math.cos(theta[1])))
    # else: 
    #     theta[1] = jointAngle[1]
    #     theta[0] = jointAngle[0]

    # Jacobian
    # dx/dt = | -l1s1 - l1s12   -l2s12  | | theta1_dot |
    # dy/dt = |  l1c1 + l2c12    l2c12  | | theta2_dot |

    l1 = jointLength[0]
    l2 = jointLength[1]
    jointAngle[1] = jointAngle[1] + 0.00001
    s1 = math.sin(jointAngle[0])
    c1 = math.cos(jointAngle[0])
    s12 = math.sin(jointAngle[0]+jointAngle[1])
    c12 = math.cos(jointAngle[0]+jointAngle[1])    

    Error = [0, 0, 0]
    Error[0] = linkPos[2][0] - x
    Error[1] = linkPos[2][1] - y

    # jacobian = np.array([[-l1*s1-l1*s12, -l2*s12], [l1*c1+l2*c12, l2*c12]])
    # inverse = np.linalg.inv(jacobian)

    jacobian2 = p.calculateJacobian(pole, numJoints - 1, linkPos[2], jointAngle[:2], jointVel[:2], [1, 1])[0]
    inverse = np.linalg.pinv(jacobian2)
    # det = a*d-b*c
    
    # Error_theta = np.matmul(inverse, Error)
    # inverse = np.array([[d/det, -b/det], [-c/det, a/det]])
    theta = np.matmul(inverse, Error)
    
    for i in range(len(jointIds)-1):
        qError = theta[i]
        qDot = (qError-prev[i])/timeStep
        qInt = (qError+prev[i])/2*timeStep
        if(i==1):
            print(theta[i])
        
        torques = -kp*theta[i]-kd*qDot
        
        # p.setJointMotorControl2(pole,jointIds[i],p.TORQUE_CONTROL,force = torques*timeStep)
        
        p.setJointMotorControl2(pole,jointIds[i],p.POSITION_CONTROL,jointAngle[i]+torques*timeStep)
        prev[i] = qError

