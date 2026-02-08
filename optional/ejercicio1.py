import pybullet as p
import pybullet_data
import argparse
import time

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)

planeId = p.loadURDF("plane.urdf")

startPos = [0,0,0]
startOrientation = p.getQuaternionFromEuler([0,0,0])

robotId = p.loadURDF("robot.urdf",startPos, startOrientation)

# Joints: 
# 0 → joint_vertical 
# 1 → joint_arm

slider_vert = p.addUserDebugParameter("Vertical rotation", -3.14, 3.14, 0) 
slider_horz = p.addUserDebugParameter("Horizontal rotation", -3.14, 3.14, 0)

for i in range (10000):
    ang1 = p.readUserDebugParameter(slider_vert) 
    ang2 = p.readUserDebugParameter(slider_horz) 
    
    p.setJointMotorControl2(robotId, 0, p.POSITION_CONTROL, ang1) 
    p.setJointMotorControl2(robotId, 1, p.POSITION_CONTROL, ang2) 

    p.stepSimulation()
    time.sleep(1./240.)
	
p.disconnect()    
