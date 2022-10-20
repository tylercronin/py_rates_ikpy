

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
#code here
