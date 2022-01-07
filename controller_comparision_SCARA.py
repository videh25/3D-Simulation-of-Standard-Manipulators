from Assignment6_7.SCARA.SCARA_raw import SCARAManipulator
import Task2 as T2
import matplotlib.pyplot as plt
import numpy as np
import sympy as sp
from SCARA.SCARA_with_noise import*

q = [np.array([-0.47954266,  1.25686521,  0.4       ]), np.array([-0.47972442,  1.25692738,  0.4       ]), np.array([-0.48025951,  1.25711007,  0.4       ]), np.array([-0.48113247,  1.25740707,  0.4       ]), np.array([-0.48232759,  1.25781156,  0.4       ]), np.array([-0.48382898,  1.25831623,  0.4       ]), np.array([-0.48562057,  1.25891339,  0.4       ]), np.array([-0.48768617,  1.259595  ,  0.4       ]), np.array([-0.49000949,  1.26035284,  0.4       ]), np.array([-0.49257418,  1.26117852,  0.4       ]), np.array([-0.49536385,  1.26206358,  0.4       ]), np.array([-0.49836211,  1.26299959,  0.4       ]), np.array([-0.50155258,  1.26397816,  0.4       ]), np.array([-0.50491891,  1.26499107,  0.4       ]), np.array([-0.50844485,  1.26603026,  0.4    
   ]), np.array([-0.51211419,  1.26708793,  0.4       ]), np.array([-0.51591087,  1.26815656,  0.4       ]), np.array([-0.51981892,  1.26922899,  0.4       ]), np.array([-0.52382253,  1.27029841,  0.4       ]), np.array([-0.52790605,  1.27135844,  0.4       ]), np.array([-0.532054  ,  1.27240311,  0.4       ]), np.array([-0.53625107,  1.27342694,  0.4       ]), np.array([-0.54048216,  1.27442491,  0.4       ]), np.array([-0.54473238,  1.27539254,  0.4       ]), np.array([-0.54898705,  1.27632581,  0.4       ]), np.array([-0.5532317 ,  1.27722126,  0.4       ]), np.array([-0.5574521 ,  1.27807594,  0.4       ]), np.array([-0.56163424,  1.27888741,  0.4       ]), np.array([-0.56576435,  1.27965377,  0.4       ]), np.array([-0.56982888,  1.2803736 ,  0.4       ]), np.array([-0.5738145,  1.281046 ,  0.4      ]), np.array([-0.57770813,  1.28167052,  0.4       
]), np.array([-0.58149687,  1.28224718,  0.4       ]), np.array([-0.58516804,  1.28277643,  0.4       ]), np.array([-0.58870915,  1.28325909,  0.4       ]), np.array([-0.59210791,  1.28369638,  0.4       ]), np.array([-0.59535216,  1.2840898 ,  0.4       ]), np.array([-0.5984299 ,  1.28444115,  0.4       ]), np.array([-0.60132926,  1.28475248,  0.4       ]), np.array([-0.60403846,  1.28502598,  0.4       ]), np.array([-0.60654578, 
 1.28526398,  0.4       ]), np.array([-0.60883958,  1.28546888,  0.4       ]), np.array([-0.61090818,  1.28564308,  0.4       ]), np.array([-0.61273992,  1.28578887,  0.4       ]), np.array([-0.61432305,  1.28590845,  0.4       ]), np.array([-0.61564575,  1.28600376,  0.4       ]), np.array([-0.61669603,  1.28607645,  0.4 
      ]), np.array([-0.61746174,  1.28612776,  0.4       ]), np.array([-0.61793048,  1.28615848,  0.4       
]), np.array([-0.6180896 ,  1.28616878,  0.4       ])]

q_dot = [sp.Matrix([
[  1.3099087345286e-17],
[-4.48232090817291e-18],
[                    0]]), sp.Matrix([
[-0.00353760704941909],
[ 0.00120942240485485],
[                   0]]), sp.Matrix([
[-0.00692492111952695],
[ 0.00236114140263459],
[                   0]]), sp.Matrix([
[-0.0101594370863136],
[0.00344884137460711],
[                  0]]), sp.Matrix([
[ -0.013239031216679],
[0.00446719764148344],
[                  0]]), sp.Matrix([
[-0.0161619416884646],
[ 0.0054118345122928],
[                  0]]), sp.Matrix([
[-0.0189267500596159],
[0.00627928219958484],
[                  0]]), sp.Matrix([
[-0.0215323635617542],
[ 0.0070669327567962],
[                  0]]), sp.Matrix([
[-0.0239779981143227],
[0.00777299520473395],
[                  0]]), sp.Matrix([
[-0.0262631619707812],
[0.00839645001825223],
[                  0]]), sp.Matrix([
[-0.0283876399187571],
[0.00893700314220434],
[                  0]]), sp.Matrix([
[-0.0303514779623396],
[ 0.0093950396985984],
[                  0]]), sp.Matrix([
[-0.0321549684176071],
[ 0.0097715775355632],
[                  0]]), sp.Matrix([
[-0.033798635352768],
[0.0100682207542292],
[                 0]]), sp.Matrix([
[-0.0352832203026926],
[ 0.0102871133328873],
[                  0]]), sp.Matrix([
[-0.0366096681848292],
[ 0.0104308929496765],
[                  0]]), sp.Matrix([
[-0.0377791133401325],
[ 0.0105026450863425],
[                  0]]), sp.Matrix([
[-0.0387928656192618],
[ 0.0105058574769817],
[                  0]]), sp.Matrix([
[-0.0396523964313905],
[ 0.0104443749476823],
[                  0]]), sp.Matrix([
[-0.0403593246709105],
[ 0.0103223546760587],
[                  0]]), sp.Matrix([
[-0.0409154024364198],
[ 0.0101442218841648],
[                  0]]), sp.Matrix([
[-0.0413225004568908],
[0.00991462596441713],
[                  0]]), sp.Matrix([
[-0.0415825931419758],
[0.00963839702609736],
[                  0]]), sp.Matrix([
[-0.0416977431771164],
[0.00932050283980595],
[                  0]]), sp.Matrix([
[-0.0416700855895025],
[0.00896600614891033],
[                  0]]), sp.Matrix([
[-0.0415018112179517],
[0.00858002231052753],
[                  0]]), sp.Matrix([
[-0.0411951495283733],
[0.00816767722381197],
[                  0]]), sp.Matrix([
[-0.0407523507265372],
[0.00773406550017909],
[                  0]]), sp.Matrix([
[-0.0401756671312429],
[0.00728420882845272],
[                  0]]), sp.Matrix([
[-0.0394673337835143],
[0.00682301448764207],
[                  0]]), sp.Matrix([
[-0.0386295482809617],
[ 0.0063552339609892],
[                  0]]), sp.Matrix([
[-0.0376644498407706],
[ 0.0058854216069475],
[                  0]]), sp.Matrix([
[-0.0365740976097104],
[0.00541789334571286],
[                  0]]), sp.Matrix([
[-0.0353604482549338],
[0.00495668532370708],
[                  0]]), sp.Matrix([
[-0.0340253328849889],
[0.00450551252287375],
[                  0]]), sp.Matrix([
[-0.0325704333662304],
[0.00406772728666728],
[                  0]]), sp.Matrix([
[-0.0309972581155535],
[ 0.0036462777400583],
[                  0]]), sp.Matrix([
[-0.0293071174659545],
[0.00324366608660745],
[                  0]]), sp.Matrix([
[-0.0275010987167305],
[0.00286190677151649],
[                  0]]), sp.Matrix([
[-0.0255800409950583],
[0.00250248450537857],
[                  0]]), sp.Matrix([
[-0.0235445100701529],
[0.00216631214891962],
[                  0]]), sp.Matrix([
[-0.0213947732751039],
[0.00185368846412059],
[                  0]]), sp.Matrix([
[-0.0191307747047368],
[0.00156425574147821],
[                  0]]), sp.Matrix([
[-0.0167521108703517],
[0.00129695731649639],
[                  0]]), sp.Matrix([
[-0.0142580070038528],
[0.00104999499047275],
[                  0]]), sp.Matrix([
[ -0.0116472942144669],
[0.000820786370876873],
[                   0]]), sp.Matrix([
[-0.00891838771081544],
[0.000605922144698355],
[                   0]]), sp.Matrix([
[-0.00606926630936023],
[0.000401123293629073],
[                   0]]), sp.Matrix([
[-0.00309745345694303],
[ 0.00020119825234934],
[                   0]]), sp.Matrix([
[-2.86636503450321e-17],
[ 1.85082225782658e-18],
[                    0]])]

q_dot2 = [sp.Matrix([
[-0.0353958269322742],
[ 0.0121119472630815],
[                  0]]),sp.Matrix([
[-0.0339365265462896],
[ 0.0115807378030542],
[                  0]]),sp.Matrix([
[-0.0324507408825162],
[ 0.0109826456899054],
[                  0]]),sp.Matrix([
[-0.0309423064806515],
[ 0.0103275836398299],
[                  0]]),sp.Matrix([
[-0.0294148638554688],
[0.00962505926987735],
[                  0]]),sp.Matrix([
[-0.0278718674924894],
[0.00888416327930345],
[                  0]]),sp.Matrix([
[-0.0263165944993666],
[0.00811355907892366],
[                  0]]),sp.Matrix([
[-0.0247521521469508],
[0.00732147401421221],
[                  0]]),sp.Matrix([
[-0.0231814844763552],
[0.00651569225548524],
[                  0]]),sp.Matrix([
[-0.0216073780969833],
[0.00570354936409099],
[                  0]]),sp.Matrix([
[-0.0200324672561608],
[0.00489192848817032],
[                  0]]),sp.Matrix([
[-0.0184592382241266],
[0.00408725809583019],
[                  0]]),sp.Matrix([
[-0.0168900330087821],
[0.00329551111761169],
[                  0]]),sp.Matrix([
[ -0.015327052392611],
[0.00252220534367003],
[                  0]]),sp.Matrix([
[-0.0137723582691964],
[0.00177240490353129],
[                  0]]),sp.Matrix([
[-0.0122278752482295],
[0.00105072264682714],
[                  0]]),sp.Matrix([
[ -0.0106953914951718],
[0.000361323241047431],
[                   0]]),sp.Matrix([
[ -0.00917655877404734],
[-0.000292073193991387],
[                    0]]),sp.Matrix([
[ -0.00767289166842269],
[-0.000906187086698138],
[                    0]]),sp.Matrix([
[-0.00618576596566075],
[-0.00147817520863402],
[                   0]]),sp.Matrix([
[-0.00471641620223586],
[-0.00200562640800375],
[                   0]]),sp.Matrix([
[-0.00326593238249528],
[-0.00248655740888093],
[                   0]]),sp.Matrix([
[-0.00183525589905072],
[-0.00291940878528939],
[                   0]]),sp.Matrix([
[-0.000425174699333483],
[ -0.00330304120087487],
[                    0]]),sp.Matrix([
[0.000963682240821317],
[-0.00363673198674018],
[                   0]]),sp.Matrix([
[ 0.00233085105987299],
[-0.00392017211251537],
[                   0]]),sp.Matrix([
[  0.0036760396779634],
[-0.00415346358921267],
[                   0]]),sp.Matrix([
[ 0.00499913488374349],
[-0.00433711732708602],
[                   0]]),sp.Matrix([
[ 0.00630020972321942],
[-0.00447205145770191],
[                   0]]),sp.Matrix([
[ 0.00757953106269234],
[-0.00455959011678675],
[                   0]]),sp.Matrix([
[ 0.00883756718911706],
[-0.00460146267314913],
[                   0]]),sp.Matrix([
[  0.0100749953042486],
[-0.00459980337905622],
[                   0]]),sp.Matrix([
[ 0.0112927087637592],
[-0.0045571514088298],
[                  0]]),sp.Matrix([
[   0.012491823909026],
[-0.00447645124508416],
[                   0]]),sp.Matrix([
[  0.0136736863374084],
[-0.00436105336593703],
[                   0]]),sp.Matrix([
[  0.0148398764564267],
[-0.00421471518170511],
[                   0]]),sp.Matrix([
[  0.0159922141681876],
[-0.00404160216610251],
[                   0]]),sp.Matrix([
[   0.017132762532529],
[-0.00384628912490859],
[                   0]]),sp.Matrix([
[  0.0182638302605633],
[-0.00363376154461793],
[                   0]]),sp.Matrix([
[  0.0193879728944628],
[-0.00340941696495094],
[                   0]]),sp.Matrix([
[  0.0205079925343977],
[-0.00317906632255105],
[                   0]]),sp.Matrix([
[  0.0216269359794593],
[-0.00294893521903394],
[                   0]]),sp.Matrix([
[0.0227480911562085],
[ -0.00272566507513],
[                 0]]),sp.Matrix([
[  0.0238749817162565],
[-0.00251631414433384],
[                   0]]),sp.Matrix([
[  0.0250113596931594],
[-0.00232835837462378],
[                   0]]),sp.Matrix([
[  0.0261611961191157],
[-0.00216969212580868],
[                   0]]),sp.Matrix([
[  0.0273286695138108],
[-0.00204862877326228],
[                   0]]),sp.Matrix([
[  0.0285181521716763],
[-0.00197390125656243],
[                   0]]),sp.Matrix([
[  0.0297341941903684],
[-0.00195466266419662],
[                   0]]),sp.Matrix([
[  0.0309815052031116],
[-0.00200048698336243],
[                   0]])]

loader = (SCARAManipulator, PID_Position_Controller)
# PD Controller
# Near the given coordinates
# For CRITICAL DAMPING
# For joint 1----
# Jeff = 58
# Beff = 2  ===> For omega_n = 5, Kp = 58*5**2 = 1450; Kd = 2*5*58 - 2 = 578
# For joint 2----
# Jeff = 17.25
# Beff = 2 ===> For omega_n = 5, Kp = 17.25*5**2 = 431; Kd = 2*5*17.25 - 2 = 170
# For joint 3----
# Jeff = 11
# Beff = 2 ===> For Kp = 1e4 ===> omega_n = 30; Kd = 2*30*11 - 2 = 658
print('STARTING PD CONTROLLER')
arkoConPD = PID_Position_Controller(arko, [1450, 431, 1e4], [0,0,0], [578, 170, 658])
ee_pos_mat_pd = []
arko.ax.plot((0.40,0.40),(0.06,0.01), (-arko.l1+0.1,-arko.l1+0.1),color = 'g', alpha=0.5)

arkoConPD.follow_trajectory(T2.q, T2.q_dot, 5, ee_pos_mat_pd)


# FeedForward Controller
from SCARA.SCARAFeedForwardController import FeedForward_Position_Controller
# Near the given coordinates
# For CRITICAL DAMPING
# For joint 1----
# Jeff = 58
# Beff = 2  ===> For omega_n = 5, Kp = 58*5**2 = 1450; Kd = 2*5*58 - 2 = 578
# For joint 2----
# Jeff = 17.25
# Beff = 2 ===> For omega_n = 5, Kp = 17.25*5**2 = 431; Kd = 2*5*17.25 - 2 = 170
# For joint 3----
# Jeff = 11
# Beff = 2 ===> For Kp = 1e4 ===> omega_n = 30; Kd = 2*30*11 - 2 = 658
print('STARTING Feed Forward CONTROLLER')
ee_pos_mat_ff = []
arkoConFF = FeedForward_Position_Controller(arko, [1450, 431, 1e4], [0,0,0], [578, 170, 658])
arko.ax.plot((0.40,0.40),(0.06,0.01), (-arko.l1+0.1,-arko.l1+0.1),color = 'g', alpha=0.5)

# FFc.arkoCon.Achieve_EE_Position((0.4,0.06,-0.4))

arkoConFF.follow_trajectory(T2.q, T2.q_dot, T2.q_dot2, 5, ee_pos_mat_ff)


# Computed Torques Controller
from SCARA.SCARAComputedTorqueFFController import ComputedTorque_FF_Position_Controller
# Near the given coordinates
# For CRITICAL DAMPING
# For joint 1----
# Jeff = 58
# Beff = 2  ===> For omega_n = 5, Kp = 58*5**2 = 1450; Kd = 2*5*58 - 2 = 578
# For joint 2----
# Jeff = 17.25
# Beff = 2 ===> For omega_n = 5, Kp = 17.25*5**2 = 431; Kd = 2*5*17.25 - 2 = 170
# For joint 3----
# Jeff = 11
# Beff = 2 ===> For Kp = 1e4 ===> omega_n = 30; Kd = 2*30*11 - 2 = 658
print('STARTING Computed Torques CONTROLLER')
arkoConCT = ComputedTorque_FF_Position_Controller(arko, [1450, 431, 1e4], [0,0,0], [578, 170, 658])
ee_pos_mat_ct = []
arko.ax.plot((0.40,0.40),(0.06,0.01), (-arko.l1+0.1,-arko.l1+0.1),color = 'g', alpha=0.5)

# CTc.arkoCon.Achieve_EE_Position((0.4,0.06,-0.4))
arkoConCT.follow_trajectory(T2.q, T2.q_dot, T2.q_dot2, 5, ee_pos_mat_ct)

# Multivariable Controller
from SCARA.SCARAMultivariableController import Multivariable_Position_Controller
print('STARTING Multivariable Controls CONTROLLER')
arkoConMC = Multivariable_Position_Controller(arko, 10, 6)
ee_pos_mat_mc = []
arko.ax.plot((0.40,0.40),(0.06,0.01), (-arko.l1+0.1,-arko.l1+0.1),color = 'g', alpha=0.5)

# Mc.arkoCon.Achieve_EE_Position((0.4,0.06,-0.4))
arkoConMC.follow_trajectory(T2.q, T2.q_dot, T2.q_dot2, 5, ee_pos_mat_mc)

plt.ioff()
fig, (ax1,ax2,ax3) = plt.subplots(3,1)

ax1.plot(T2.T1.t, [np.squeeze(np.asarray(pos))[0] for pos in T2.T1.position], label = 'Desired Path')
ax1.plot(np.linspace(0,5,532), [pos[0] for pos in ee_pos_mat_pd], label = 'PD')
ax1.plot(np.linspace(0,5,532), [pos[0] for pos in ee_pos_mat_ff], label = 'FeedForward')
ax1.plot(np.linspace(0,5,532), [pos[0] for pos in ee_pos_mat_ct], label = 'Computed Torques')
ax1.plot(np.linspace(0,5,532), [pos[0] for pos in ee_pos_mat_mc], label = 'Multivariable Control')
ax1.set_title('X coordinates')
ax1.legend()
ax1.grid()

ax2.plot(T2.T1.t, [np.squeeze(np.asarray(pos))[1] for pos in T2.T1.position], label = 'Desired Path')
ax2.plot(np.linspace(0,5,532), [pos[1] for pos in ee_pos_mat_pd], label = 'PD')
ax2.plot(np.linspace(0,5,532), [pos[1] for pos in ee_pos_mat_ff], label = 'FeedForward')
ax2.plot(np.linspace(0,5,532), [pos[1] for pos in ee_pos_mat_ct], label = 'Computed Torques')
ax2.plot(np.linspace(0,5,532), [pos[1] for pos in ee_pos_mat_mc], label = 'Multivariable Control')
ax2.set_title('Y coordinates')
ax2.legend()
ax2.grid()

ax3.plot(T2.T1.t, [np.squeeze(np.asarray(pos))[2] for pos in T2.T1.position], label = 'Desired Path')
ax3.plot(np.linspace(0,5,532), [0.5+pos[2] for pos in ee_pos_mat_pd], label = 'PD')
ax3.plot(np.linspace(0,5,532), [0.5+pos[2] for pos in ee_pos_mat_ff], label = 'FeedForward')
ax3.plot(np.linspace(0,5,532), [0.5+pos[2] for pos in ee_pos_mat_ct], label = 'Computed Torques')
ax3.plot(np.linspace(0,5,532), [0.5+pos[2] for pos in ee_pos_mat_mc], label = 'Multivariable Control')
ax3.set_title('Z coordinates')
ax3.legend()
ax3.grid()

plt.show()

fig,ax = plt.subplots()
ax.plot([np.squeeze(np.asarray(pos))[0] for pos in T2.T1.position], [np.squeeze(np.asarray(pos))[1] for pos in T2.T1.position], label = 'Desired Path')
ax.plot([pos[0] for pos in ee_pos_mat_pd], [pos[1] for pos in ee_pos_mat_pd], label = 'PD')
ax.plot([pos[0] for pos in ee_pos_mat_ff], [pos[1] for pos in ee_pos_mat_ff], label = 'FeedForward')
ax.plot([pos[0] for pos in ee_pos_mat_ct], [pos[1] for pos in ee_pos_mat_ct], label = 'Computed Torques')
ax.plot([pos[0] for pos in ee_pos_mat_mc], [pos[1] for pos in ee_pos_mat_mc], label = 'Multivariable Control')
ax.set_title('X-Y Plane View')
ax.legend()
ax.grid()
plt.show()
