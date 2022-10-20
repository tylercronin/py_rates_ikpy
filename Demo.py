

##---------------------------------- Demonstration 1---------------------------------


# IKPy package installed through anaconda successfully using pip

from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Manual creation of chain with origin link, four motor links, and an end link

my_chain = Chain(name='link 1', links=[
    OriginLink(),
    URDFLink(
      name="link 2",
      origin_translation=[-10, 0, 5],
      origin_orientation=[0, 0, 0],
      rotation=[0, 1, 0],
    ),
    URDFLink(
      name="link 3",
      origin_translation=[10, 0, 0],
      origin_orientation=[0, 0, 0],
      rotation=[0, 1, 0],
    ),
    URDFLink(
      name="link 4",
      origin_translation=[20, 0, 0],
      origin_orientation=[0, 0, 0],
      rotation=[0, 1, 0],
    ),
    URDFLink(
      name="link 5",
      origin_translation=[30, 0, 0],
      origin_orientation=[0, 0, 0],
      rotation=[0, 1, 0],
    ),
      URDFLink(
        name="link 6",
        origin_translation=[40, 0, 0],
        origin_orientation=[0, 0, 0],
        rotation=[0, 1, 0],
    )
])

target_position = [40, 5, 20]


chain_position = my_chain.forward_kinematics([0] * 6)
start_effector_orientation = chain_position[:3, :3]
start_effector_position = chain_position[:3, 3]
print("Position of end-effector before transformation: ", start_effector_position )
print("Orientation of end-effector before transformation: ", "\n",start_effector_orientation,"\n")

transformed_chain = my_chain.inverse_kinematics(target_position)

new_chain_position = my_chain.forward_kinematics(my_chain.inverse_kinematics(target_position))
end_effector_orientation = new_chain_position[:3, :3]
end_effector_position = new_chain_position[:3, 3]

#end_effector_orientation = chain_position[:3, :3]
#end_effector_position = chain_position[:3, 3]
print("Position of end-effector after transformation: ", end_effector_position )
print("Orientation of end-effector after transformation: ", "\n",end_effector_orientation)

ax = plt.figure().add_subplot(111, projection='3d')

my_chain.plot(transformed_chain, ax)
plt.show()



##---------------------------------- Demonstration 2---------------------------------


import ikpy.chain
import ikpy.utils.plot as plot_utils
import matplotlib.pyplot as plt

plt.close('all')

my_chain = ikpy.chain.Chain.from_urdf_file("//Users/yugthapa/Downloads/Application Programing for Eng/Lightining Talk/ikpy-3.3.3/resources/poppy_ergo.URDF")
target_position = [ 0.1, -0.5, 0.1]
print("The angles of each joints are : ", my_chain.inverse_kinematics(target_position))
real_frame = my_chain.forward_kinematics(my_chain.inverse_kinematics(target_position))
print("Computed position vector : %s, original position vector : %s" % (real_frame[:3, 3], target_position))
fig, ax = plot_utils.init_3d_figure()
my_chain.plot(my_chain.inverse_kinematics(target_position), ax, target=target_position)
plt.xlim(-0.5, 0.5)
plt.ylim(-0.5, 0.5)
plt.show()


##-------------------------------------Demonstration 3---------------------------------
import numpy as np

from ikpy.chain import Chain
from ikpy.utils import plot

baxter_left_arm_chain = Chain.from_json_file("resources/baxter/baxter_left_arm.json")
baxter_right_arm_chain = Chain.from_json_file("resources/baxter/baxter_right_arm.json")
baxter_pedestal_chain = Chain.from_json_file("resources/baxter/baxter_pedestal.json")
baxter_head_chain = Chain.from_json_file("resources/baxter/baxter_head.json")


from mpl_toolkits.mplot3d import Axes3D;
fig, ax = plot.init_3d_figure();
baxter_left_arm_chain.plot([0] * (len(baxter_left_arm_chain)), ax)
baxter_right_arm_chain.plot([0] * (len(baxter_right_arm_chain)), ax)
#baxter_pedestal_chain.plot([0] * (2 + 2), ax)
#baxter_head_chain.plot([0] * (4 + 2), ax)
ax.legend()


# Let's ask baxter to put his left arm at a target_position, with a target_orientation on the X axis.
# This means we want the X axis of his hand to follow the desired vector
target_orientation = [0, 0, 1]
target_position = [0.1, 0.5, -0.1]

# Compute the inverse kinematics with position
ik = baxter_left_arm_chain.inverse_kinematics(target_position, target_orientation, orientation_mode="X")

# Let's see what are the final positions and orientations of the robot
position = baxter_left_arm_chain.forward_kinematics(ik)[:3, 3]
orientation = baxter_left_arm_chain.forward_kinematics(ik)[:3, 0]

# And compare them with was what required
print("Requested position: {} vs Reached position: {}".format(target_position, position))
print("Requested orientation on the X axis: {} vs Reached orientation on the X axis: {}".format(target_orientation, orientation))
# We see that the chain reached its position!

# Plot how it goes
fig, ax = plot.init_3d_figure();
baxter_left_arm_chain.plot(ik, ax)
baxter_right_arm_chain.plot([0] * (len(baxter_right_arm_chain)), ax)
#baxter_pedestal_chain.plot([0] * (2 + 2), ax)
#baxter_head_chain.plot([0] * (4 + 2), ax)
ax.legend()


# Let's ask baxter to put his left arm's X axis to the absolute Z axis
orientation_axis = "X"
target_orientation = [0, 0, 1]

# Compute the inverse kinematics with position
ik = baxter_left_arm_chain.inverse_kinematics(
    target_position=[0.1, 0.5, -0.1],
    target_orientation=target_orientation,
    orientation_mode=orientation_axis)

# Plot how it goes
fig, ax = plot.init_3d_figure();
baxter_left_arm_chain.plot(ik, ax)
baxter_right_arm_chain.plot([0] * (len(baxter_right_arm_chain)), ax)
#baxter_pedestal_chain.plot([0] * (2 + 2), ax)
#baxter_head_chain.plot([0] * (4 + 2), ax)
ax.legend()

# Let's ask baxter to put his left arm as 
target_orientation = np.eye(3)

# Compute the inverse kinematics with position
ik = baxter_left_arm_chain.inverse_kinematics(
    target_position=[0.1, 0.5, -0.1],
    target_orientation=target_orientation,
    orientation_mode="all")

# Plot how it goes
fig, ax = plot.init_3d_figure();
baxter_left_arm_chain.plot(ik, ax)
baxter_right_arm_chain.plot([0] * (len(baxter_right_arm_chain)), ax)
#baxter_pedestal_chain.plot([0] * (2 + 2), ax)
#baxter_head_chain.plot([0] * (4 + 2), ax)
ax.legend()


