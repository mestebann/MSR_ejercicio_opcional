import pybullet as p
import pybullet_data
import time

urdf_path = "opcional.urdf"

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8) 

planeId = p.loadURDF("plane.urdf")

startPosition = [0,0,1]
euler_angles = [0,0,0]
startOrientation = p.getQuaternionFromEuler(euler_angles)

robotId = p.loadURDF(urdf_path, startPosition, startOrientation)

numJoints = p.getNumJoints(robotId)
print("NumJoints: {}".format(numJoints))
for j in range(numJoints):
    print("{} - {}".format(p.getJointInfo(robotId,j)[0], p.getJointInfo(robotId,j)[1].decode("utf-8")))

right_id = p.addUserDebugParameter("motorHorizontal", -1, 1, 0)
left_id = p.addUserDebugParameter("motorVertical", -1, 1, 0)

try:
    while True:
        p.stepSimulation()
        time.sleep(1./240.)

        right_motor_value = p.readUserDebugParameter(right_id)
        left_motor_value = p.readUserDebugParameter(left_id)
        
        p.setJointMotorControl2(robotId,1, p.VELOCITY_CONTROL, targetVelocity=right_motor_value)
        p.setJointMotorControl2(robotId,0, p.VELOCITY_CONTROL, targetVelocity=left_motor_value)

except KeyboardInterrupt:
    pass

p.disconnect()
