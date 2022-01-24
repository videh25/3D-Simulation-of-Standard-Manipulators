from load import*

#############################################
print("Stanford Manipulator")
arko_Stanford,arkoCon_PID = load(StanfordManipulator, PID_Position_Controller)
print(" - PID Controller")
arkoCon_PID.Achieve_EE_Position((2,3,0))

print(" - FeedForward Controller")
arko_Stanford.reset()
arkoCon_FF = FeedForward_Position_Controller(arko_Stanford)
arkoCon_FF.Achieve_EE_Position((2,3,0))

print(" - Computed Torques Controller")
arko_Stanford.reset()
arkoCon_CT_FF = ComputedTorque_FF_Position_Controller(arko_Stanford)
arkoCon_CT_FF.Achieve_EE_Position((2,3,0))

print(" - Multivariable Controller")
arko_Stanford.reset()
arkoCon_MVC = Multivariable_Position_Controller(arko_Stanford)
arkoCon_MVC.Achieve_EE_Position((2,3,0))

arko_Stanford.stop()

#############################################
print("PUMA Manipulator")
arko_PUMA,arkoCon_PID = load(PUMAManipulator, PID_Position_Controller)
print(" - PID Controller")
arkoCon_PID.Achieve_EE_Position((0,5,7))

print(" - FeedForward Controller")
arko_PUMA.reset()
arkoCon_FF = FeedForward_Position_Controller(arko_PUMA)
arkoCon_FF.Achieve_EE_Position((0,5,7))

print(" - Computed Torques Controller")
arko_PUMA.reset()
arkoCon_CT_FF = ComputedTorque_FF_Position_Controller(arko_PUMA)
arkoCon_CT_FF.Achieve_EE_Position((0,5,7))

print(" - Multivariable Controller")
arko_PUMA.reset()
arkoCon_MVC = Multivariable_Position_Controller(arko_PUMA)
arkoCon_MVC.Achieve_EE_Position((0,5,7))

arko_PUMA.stop()

#############################################
print("SCARA Manipulator")
arko_SCARA,arkoCon_PID = load(SCARAManipulator, PID_Position_Controller)
print(" - PID Controller")
arkoCon_PID.Achieve_EE_Position((0.4,0.06,-0.3))

print(" - FeedForward Controller")
arko_SCARA.reset()
arkoCon_FF = FeedForward_Position_Controller(arko_SCARA)
arkoCon_FF.Achieve_EE_Position((0.4,0.06,-0.3))

print(" - Computed Torques Controller")
arko_SCARA.reset()
arkoCon_CT_FF = ComputedTorque_FF_Position_Controller(arko_SCARA)
arkoCon_CT_FF.Achieve_EE_Position((0.4,0.06,-0.3))

print(" - Multivariable Controller")
arko_SCARA.reset()
arkoCon_MVC = Multivariable_Position_Controller(arko_SCARA)
arkoCon_MVC.Achieve_EE_Position((0.4,0.06,-0.3))

arko_SCARA.stop()