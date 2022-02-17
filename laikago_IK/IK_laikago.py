import pybullet as p
import numpy as np
import time

timeStep = 1./500.
p.connect(p.GUI)
plane = p.loadURDF("plane.urdf")
p.setGravity(0,0,-9.8)
p.setTimeStep(timeStep)
#p.setDefaultContactERP(0)
#urdfFlags = p.URDF_USE_SELF_COLLISION+p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS 
urdfFlags = p.URDF_USE_SELF_COLLISION
quadruped = p.loadURDF("laikago/laikago_toes.urdf",[0,0,.5],[0,0.5,0.5,0], flags = urdfFlags,useFixedBase=True)

#enable collision between lower legs

# for j in range (p.getNumJoints(quadruped)):
# 		print(p.getJointInfo(quadruped,j))

#2,5,8 and 11 are the lower legs
lower_legs = [2,5,8,11]
for l0 in lower_legs:
	for l1 in lower_legs:
		if (l1>l0):
			enableCollision = 1
			print("collision for pair",l0,l1, p.getJointInfo(quadruped,l0)[12],p.getJointInfo(quadruped,l1)[12], "enabled=",enableCollision)
			p.setCollisionFilterPair(quadruped, quadruped, l0,l1,enableCollision)

jointIds=[]
paramIds=[]
jointOffsets=[]
jointDirections=[-1,1,1,1,1,1,-1,1,1,1,1,1]
jointAngles=[0,0,0,0,0,0,0,0,0,0,0,0]

for i in range (4):
	jointOffsets.append(0)
	jointOffsets.append(-0.7)
	jointOffsets.append(0.7)

maxForceId = p.addUserDebugParameter("maxForce",0,100,20)

for j in range (p.getNumJoints(quadruped)):
        p.changeDynamics(quadruped,j,linearDamping=0, angularDamping=0)
        info = p.getJointInfo(quadruped,j)
        #print(info)
        jointName = info[1]
        jointType = info[2]
        if (jointType==p.JOINT_PRISMATIC or jointType==p.JOINT_REVOLUTE):
                jointIds.append(j)

		
p.getCameraImage(480,320)
p.setRealTimeSimulation(0)

joints=[]

# with open("data1.txt","r") as filestream:
# 	for line in filestream:
# 		maxForce = p.readUserDebugParameter(maxForceId)
# 		currentline = line.split(",")
# 		frame = currentline[0]
# 		t = currentline[1]
# 		joints=currentline[2:14]
# 		for j in range (12):
# 			targetPos = float(joints[j])
# 			p.setJointMotorControl2(quadruped,jointIds[j],p.POSITION_CONTROL,jointDirections[j]*targetPos+jointOffsets[j], force=maxForce)
# 		p.stepSimulation()
# 		for lower_leg in lower_legs:
# 			#print("points for ", quadruped, " link: ", lower_leg)
# 			pts = p.getContactPoints(quadruped,-1, lower_leg)
# 			#print("num points=",len(pts))
# 			#for pt in pts:
# 			#	print(pt[9])
# 		time.sleep(1./500.)


# control 3 = toeFR by 0 1 2 
# control 7 = toeFL by 4 5 6
# control 11 = toeRR by 8 9 10
# control 15 = toeRL by 12 13 14
endEffector = [3, 7, 11, 15]
initPos = []
for j in endEffector:
    link = p.getLinkState(quadruped,j)
    # print(link)
    x = link[4][0]
    y = link[4][1]
    z = link[4][2]
    initPos.append(link[4])

    info = p.getJointInfo(quadruped,j)
    jointName = info[1]
    paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8")+"x", x-1, x+1, x))
    paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8")+"y", y-1, y+1, y))
    paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8")+"z", z-1, z+1, z))


index = 0
for j in range (p.getNumJoints(quadruped)):
    p.changeDynamics(quadruped,j,linearDamping=0, angularDamping=0)
    info = p.getJointInfo(quadruped,j)
    js = p.getJointState(quadruped,j)
    #print(info)
    jointName = info[1]
    jointType = info[2]




def accurateIK(endEffectorId, targetPosition, maxForce):

    numJoints = p.getNumJoints(quadruped)

    ls = p.getLinkState(quadruped,endEffectorId)    
    newPos = ls[4]

    jointPoses = p.calculateInverseKinematics(quadruped, endEffectorId, targetPosition)
    print(len(jointPoses))
    
    j = 0
    for i in range(numJoints):
        # p.resetJointState(quadruped, i, theta[i])
        if i%4 != 3:
            p.resetJointState(quadruped, i, jointPoses[j])
            j = j+1
    return

        
p.setRealTimeSimulation(1)

while (1):
    j = 0
    for i in endEffector:
        c = paramIds[j]
        targetPosX = p.readUserDebugParameter(c)
        j = j+1
        c = paramIds[j]
        targetPosY = p.readUserDebugParameter(c)
        j = j+1
        c = paramIds[j]
        targetPosZ = p.readUserDebugParameter(c)
        j = j+1
        maxForce = p.readUserDebugParameter(maxForceId)
        targetPos = [targetPosX, targetPosY, targetPosZ]
        accurateIK(i, targetPos, maxForce)

        # p.setJointMotorControl2(quadruped,jointIds[i],p.POSITION_CONTROL,jointDirections[i]*targetPos+jointOffsets[i], force=maxForce)
		