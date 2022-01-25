from Robots import *
from Controllers import * 

print("ROBOTS AVAILABLE: \n  -PUMAManipulator \n  -SCARAManipulator \n  -StanfordManipulator")
print("CONTROLLERS AVAILABLE: \n  -PID_Position_Controller  \n  -ComputedTorque_FF_Position_Controller \n  -FeedForward_Position_Controller \n  -Multivariable_Position_Controller")

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
