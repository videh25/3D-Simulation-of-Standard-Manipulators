from Robots import *
from Controllers import * 

print("INFO:: Robots available: PUMAManipulator; SCARAManipulator; StanfordManipulator")
print("INFO:: Controllers available: PID_Position_Controller;  ComputedTorque_FF_Position_Controller; FeedForward_Position_Controller; Multivariable_Position_Controller")

def load(robot_class, controller_class = None):
    if controller_class is not None:
        robo = robot_class()
        robocon = controller_class(robo)
        robo.run()
        return robo, robocon
    else:
        robo = robot_class()
        
        robo.run()
        return robo
