import tf

x = -0.030422068810453017
y = -0.03589244627821691
z = -0.9192435142037735
w = 0.39086774185618683


#type(pose) = geometry_msgs.msg.Pose
quaternion = (
    x,
    y,
    z,
    w)
euler = tf.transformations.euler_from_quaternion(quaternion)
roll = euler[0]
pitch = euler[1]
yaw = euler[2]

print(roll,pitch,yaw)