
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