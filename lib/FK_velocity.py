
import numpy as np 
from lib.calcJacobian import calcJacobian

def FK_velocity(q_in, dq):
    """
    :param q_in: 1 x 7 vector corresponding to the robot's current configuration.
    :param dq: 1 x 7 vector corresponding to the joint velocities.
    :return:
    velocity - 6 x 1 vector corresponding to the end effector velocities.    
    """

    ## STUDENT CODE GOES HERE
    J=np.round(calcJacobian(q_in),3)        
    velocity = np.zeros((6, 1))
    velocity= np.dot(J,dq)

    return velocity