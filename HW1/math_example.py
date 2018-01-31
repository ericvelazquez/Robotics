from RobotLib.Math import *
import math

# set child frame parameters
cx = 10
cy = 20
theta = 45.*(math.pi/180.)

# create transformation matrices
T_child_parent = transform(cx,cy,theta)
T_parent_child = invert(T_child_parent)

# create point in child frame
p_child = vec(5,5)

# transform to parent frame
p_parent = mul(T_child_parent,p_child)

print('inverse test: ')
print(T_child_parent*T_parent_child)
print(T_parent_child*T_child_parent)

print('point in child frame: ')
print(p_child)

print('point in parent frame: ')
print(p_parent)

# make grandchild frame parameters
cx = 5
cy = 10
theta = 20.*(math.pi/180.)

# make grandchild transformation matrix
T_grandchild_child  = transform(cx,cy,theta)

# create point in grandchild frame
p_grandchild = vec(10,10)

# get point in parent frame
p_parent = mul(T_child_parent*T_grandchild_child,p_grandchild)

print('point in grandchild frame: ')
print(p_grandchild)

print('point in parent frame: ')
print(p_parent)
