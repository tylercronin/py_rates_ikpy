

##---------------------------------- Demonstration 1---------------------------------
#code here









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
