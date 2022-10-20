

from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import matplotlib.pyplot as plt
import ikpy.utils.plot as plot_utils
import numpy as np
from ikpy.utils import plot
import ikpy.chain


##---------------------------------- Demonstration 1---------------------------------
my_chain = Chain(name='link 1', links=[
    OriginLink(),
    URDFLink(
      name="link 2",
      origin_translation=[0, 0, 5],
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

target_position = [40, 0, 20]


chain_position = my_chain.forward_kinematics([0] * 6)
start_effector_orientation = chain_position[:3, :3]
start_effector_position = chain_position[:3, 3]



transformed_chain = my_chain.inverse_kinematics(target_position)

new_chain_position = my_chain.forward_kinematics(my_chain.inverse_kinematics(target_position))
end_effector_orientation = new_chain_position[:3, :3]
end_effector_position = new_chain_position[:3, 3]



ax = plt.figure().add_subplot(111, projection='3d')

my_chain.plot(transformed_chain, ax)
plt.show()

##---------------------------------- Demonstration 2---------------------------------

my_chain = ikpy.chain.Chain.from_urdf_file("//Users/yugthapa/Downloads/Application Programing for Eng/Lightining Talk/ikpy-3.3.3/resources/poppy_ergo.URDF")
target_position = [ 0.1, -0.1, 0.1]
real_frame = my_chain.forward_kinematics(my_chain.inverse_kinematics(target_position))
fig, ax = plot_utils.init_3d_figure()
my_chain.plot(my_chain.inverse_kinematics(target_position), ax, target=target_position)
plt.xlim(-0.5, 0.5)
plt.ylim(-0.5, 0.5)
plt.show()

##-------------------------------------Demonstration 3---------------------------------

# import the baxter json files
baxter_left_arm_chain = Chain.from_json_file("/Users/yugthapa/Downloads/Application Programing for Eng/Lightining Talk/py_rates_ikpy-main/baxter_left_arm.json")
baxter_right_arm_chain = Chain.from_json_file("/Users/yugthapa/Downloads/Application Programing for Eng/Lightining Talk/py_rates_ikpy-main/baxter_right_arm.json")
baxter_pedestal_chain = Chain.from_json_file("/Users/yugthapa/Downloads/Application Programing for Eng/Lightining Talk/py_rates_ikpy-main/baxter_pedestal.json")
baxter_head_chain = Chain.from_json_file("/Users/yugthapa/Downloads/Application Programing for Eng/Lightining Talk/py_rates_ikpy-main/baxter_head.json")


# plot initial position of baxter
fig, ax = plot.init_3d_figure();
baxter_left_arm_chain.plot([0] * (len(baxter_left_arm_chain)), ax)
baxter_right_arm_chain.plot([0] * (len(baxter_right_arm_chain)), ax)
ax.legend()
# target position and orientation
target_orientation = [0, 0, 1]
target_position = [0.1, 0.5, -0.1]

#IK
ik = baxter_left_arm_chain.inverse_kinematics(target_position, target_orientation, orientation_mode="X")

#Shift and orientate arms to target position 
position = baxter_left_arm_chain.forward_kinematics(ik)[:3, 3]
orientation = baxter_left_arm_chain.forward_kinematics(ik)[:3, 0]
#plot
fig, ax = plot.init_3d_figure();
baxter_left_arm_chain.plot(ik, ax)
baxter_right_arm_chain.plot([0] * (len(baxter_right_arm_chain)), ax)
ax.legend()


orientation_axis = "X"
target_orientation = [0, 0, 1]

#reorientated axis
ik = baxter_left_arm_chain.inverse_kinematics(
    target_position=[0.1, 0.5, -0.1],
    target_orientation=target_orientation,
    orientation_mode=orientation_axis)
#plotting arm
fig, ax = plot.init_3d_figure();
baxter_left_arm_chain.plot(ik, ax)
baxter_right_arm_chain.plot([0] * (len(baxter_right_arm_chain)), ax)

ax.legend()


target_orientation = np.eye(3)

#IK with all 3 axis orientation
ik = baxter_left_arm_chain.inverse_kinematics(
    target_position=[0.1, 0.5, -0.1],
    target_orientation=target_orientation,
    orientation_mode="all")

# Plotting after IK with all axis
fig, ax = plot.init_3d_figure();
baxter_left_arm_chain.plot(ik, ax)
baxter_right_arm_chain.plot([0] * (len(baxter_right_arm_chain)), ax)
ax.legend()
plt.show()


