import pybullet as p
import time
import pybullet_data
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,1]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
# boxId = p.loadURDF("r2d2.urdf",cubeStartPos, cubeStartOrientation)
# boxId = p.loadURDF("/home/yan/yxj/ws_ultrasound/src/franka_ros_ultra/franka_description/robots/panda_arm.urdf",[1,1,0.6], cubeStartOrientation,useFixedBase =1)
boxId = p.loadURDF("/home/yan/yxj/ws_ultrasound/src/franka_ros_ultra/franka_description/robots/panda_transducer_for_pybullet/panda_arm.urdf",[0,0,0])

for i in range (100000):
    p.stepSimulation()
    time.sleep(1./240.)
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos,cubeOrn)
p.disconnect()