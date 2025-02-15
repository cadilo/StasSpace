import matplotlib.pyplot

from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
from ikpy.inverse_kinematics import inverse_kinematic_optimization
from ikpy.utils.geometry import from_transformation_matrix, to_transformation_matrix

def rot2eul(R):
    beta = -np.arcsin(R[2,0])
    alpha = np.arctan2(R[2,1]/np.cos(beta),R[2,2]/np.cos(beta))
    gamma = np.arctan2(R[1,0]/np.cos(beta),R[0,0]/np.cos(beta))
    return [ np.degrees(x) for x in [alpha, beta, gamma] ]

if __name__ == '__main__':
    import numpy as np
    chain = Chain.from_urdf_file("description.urdf", active_links_mask=[0, 1, 1, 1, 1, 1, 1])
    #print(chain.links)



    print("------------------------IK---------------------")
    # to invers kinematic
    initial_joint_angels = [0, 0, 0.873, 1.265, 0, 0, 0]

    #vector = np.array([-0.80, 0.0, 0.2])
    #rot = np.array([179.9683, -67.5016, 179.9866])
    #k = to_transformation_matrix(vector, rot)

    #target_frame = k

    target_frame = np.array([
    [1, 0, 0, -0.70],
    [0, 1, 0, 0.05],
    [0, 0, 1, -0.15],
    [0, 0, 0, 1.0]
    ])

    invers_opt = inverse_kinematic_optimization(chain, target_frame, initial_joint_angels, orientation_mode="all")
    #invers = chain.inverse_kinematics(vector, rot, 'all')    
    normilized_opt = [f"{v:.2f}" for v in invers_opt]
    #normilized = [f"{v:.2f}" for v in invers]
    print("invers_opt: ", invers_opt)
    print("(invers)normilize coord pos optimization: ", normilized_opt)  
    #print("(invers)normilize coord pos: ", normilized) 


    print("----------FK----------------------")
    #to forward kinematic
    pk = chain.forward_kinematics([0, 0, 0.873, 1.265, 0, 0, 0])
    V, R = from_transformation_matrix(pk)
    print("FK pos: ", V)
    #print(R)
    A = rot2eul(R)
    print("Rotation: ", R)
    #print("FW rot:", A)
    V_new = V[:-1]
    k = to_transformation_matrix(V_new, R)
    print("matrix k: ", k)

    invers_opt = inverse_kinematic_optimization(chain, k, initial_joint_angels, orientation_mode="all")

    normilized = [f"{v:.2f}" for v in invers_opt]

    #print(k)
    print("pos joint: ", normilized)
     #invers = chain.inverse_kinematics(vector, rot, 'all')
    #print(invers)
    #print(invers)

    ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')
    chain.plot(invers_opt, ax)
    matplotlib.pyplot.show()
    
