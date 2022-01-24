#Author: Videh Patel : videh.p@iitgn.ac.in
import numpy as np
import sympy as sp

class PID_Position_Controller:
    # Simply applies individual joint PID control to achieve the target position
    def __init__(self, RRMmodel, Kp = None, Ki = None, Kd = None):
        self.Manipulator = RRMmodel

        ## Kp,Kd Calculations
        #For "SCARA":
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
            
        #For "PUMA":
            # Near the given coordinates (5,7,0)
            # For CRITICAL DAMPING
            # For joint 1----
            # Jeff = 1 + 5 = 6
            # Beff = 2  ===> For omega_n = 10, Kp = 6*10**2 = 600; Kd = 2*6*10 - 2 = 118
            # For joint 2----
            # Jeff = 31.16 + 1 = 32.16
            # Beff = 2 ===> For omega_n = 10, Kp = 32.16*10**2 = 3216; Kd = 2*10*32.16 - 2 = 641.2
            # For joint 3----
            # Jeff = 8.33 + 1 = 9.33
            # Beff = 2 ===> For omega_n = 10, Kp = 9.33*10**2 = 933; Kd = 2*10*9.33 - 2 = 186.6

        #For "Stanford":
            # Near the given coordinates (2,3,0)
            # For CRITICAL DAMPING
            # For joint 1----
            # Jeff = 1 + 4.33 = 5.33
            # Beff = 2  ===> For omega_n = 10, Kp = 5.33*10**2 = 534; Kd = 2*5.33*10 - 2 = 104.6
            # For joint 2----
            # Jeff = 12.67 + 1 = 13.67
            # Beff = 2 ===> For omega_n = 10, Kp = 13.67*10**2 = 1367; Kd = 2*10*13.67 - 2 = 271.4
            # For joint 3----
            # Jeff = 1 + 1 = 2
            # Beff = 2 ===> For omega_n = 10, Kp = 2*10**2 = 200; Kd = 2*10*2 - 2 = 38

        if Kp is None:
            if RRMmodel.type == "SCARA":
                self.Kp = np.array([1450, 431, 1e4])
            
            elif RRMmodel.type == "PUMA":
                self.Kp = np.array([150, 804, 233.25])
            
            elif RRMmodel.type == "Stanford":
                self.Kp = np.array([534, 1367, 200])
        else:
            self.Kp = np.array(Kp)
        
        if Kd is None:
            if RRMmodel.type == "SCARA":
                self.Kd = np.array([578, 170, 658])
            
            elif RRMmodel.type == "PUMA":
                self.Kd = np.array([58, 319.6, 91.3])
        
            elif RRMmodel.type == "Stanford":
                self.Kd = np.array([104.6, 271.4, 38])
        else:
            self.Kd = np.array(Kd)

        if Ki is None:
            if RRMmodel.type == "SCARA":
                self.Ki = np.array([0,0,0])
            
            elif RRMmodel.type == "PUMA":
                self.Ki = np.array([0,0,0])
        
            elif RRMmodel.type == "Stanford":
                self.Ki = np.array([0,0,0])
        else:
            self.Ki = np.array(Ki)

        self.nDOF = int(len(self.Manipulator.state)/2)
        self.target_state = np.array(2*self.nDOF*[0.])

        self.integral_error = np.array(self.nDOF*[0.])
        self.previous_error = np.array(self.nDOF*[0.])
        self.error_matrix = np.zeros((3,3))

    def set_K(self, K_p, K_i, K_d):
        self.Kp = np.array(K_p)
        self.Ki = np.array(K_i)
        self.Kd = np.array(K_d)
        self.reset()

    def reset(self):
        curr_state = self.Manipulator.get_state()[:self.nDOF]

        self.integral_error = np.array(self.nDOF*[0.])
        self.previous_error = np.array(self.target_state[:self.nDOF] - curr_state)

        self.error_matrix[0,:] = self.previous_error
        self.error_matrix[1,:] = self.previous_error
        self.error_matrix[2,:] = self.previous_error

    def set_target_state(self, state_tup):
        self.target_state = np.array(state_tup)
        self.reset()
    
    def calculate_output_voltages(self):
        curr_state = self.Manipulator.get_state()[:self.nDOF]
        error = np.array(self.target_state[:self.nDOF] - curr_state)
        # print(error)

        diff_error = self.target_state[self.nDOF:] - self.Manipulator.get_state()[self.nDOF:]
        self.integral_error = self.integral_error  + (error + self.previous_error)*self.Manipulator.dt/2

        self.previous_error = error

        self.error_matrix[0,:] = self.error_matrix[1,:]
        self.error_matrix[1,:] = self.error_matrix[2,:]
        self.error_matrix[2,:] = self.previous_error.T
        # print(self.error_matrix)

        V = ((self.Kp*error + self.Ki*self.integral_error + self.Kd*diff_error))
        return sp.Matrix(V)

    def achieve_target_state(self):
        err_mat = self.error_matrix.astype(float)
        # err_mat[:,2] = (self.error_matrix[:,2]).astype(float)*0.01
        while ((np.linalg.norm(err_mat[:3,0]) > 2e-2) or (np.linalg.norm(err_mat[:3,1]) > 3e-2) or (np.linalg.norm(err_mat[:3,2]) > 10e-2)):
            # print(np.linalg.norm(err_mat[0,:2]) + np.linalg.norm(err_mat[1,:2]) + np.linalg.norm(err_mat[2,:2])/3)
            # print((np.abs(err_mat[0,0]) + np.abs(err_mat[1,0]) + np.abs(err_mat[2,0])/3))
            # print(np.linalg.norm(self.Manipulator.state[3:].astype(float)))
            # print(err_mat)
            # print((err_mat[:3,2]))
            # print(np.linalg.norm(err_mat[:3,2]))
            # print(np.linalg.norm(err_mat[:3,0]) > 2e-2, np.linalg.norm(err_mat[:3,1]) > 3e-2, np.linalg.norm(err_mat[:3,2]) > 10e-2)
            self.Manipulator.apply_voltages(self.calculate_output_voltages())
            err_mat = self.error_matrix.astype(float)
            if (np.linalg.norm((self.Manipulator.state[self.nDOF: 2*self.nDOF]).astype(float)) < 5e-3):
                err_mat = np.zeros((3,3))



    def follow_trajectory(self, q_mat, qdot_mat, t, ee_pos_values = None):
        # t: Total time for execution of trajectory
        # q_mat: array of q's
        # qdot_mat: array of qdot's
        if len(q_mat) != len(qdot_mat):
            print('q_mat and qdot_mat have different sizes')
            return None
        dt = t/len(qdot_mat)
        if dt < self.Manipulator.dt:
            print('Array passed has too refined path. Not all points will be achieved')
        self.Manipulator.reset()
        self.Manipulator.set_state(np.array(np.squeeze(np.asarray(q_mat[0])).tolist() + np.squeeze(np.asarray(qdot_mat[0])).tolist()))
        target_start_time = 0
        for i in range(len(q_mat)):
            print('Starting to follow next point: ' + str(i))
            self.set_target_state(np.array(np.squeeze(np.asarray(q_mat[i])).tolist() + np.squeeze(np.asarray(qdot_mat[i])).tolist()))
            target_start_time = self.Manipulator.time
            while (self.Manipulator.time - target_start_time) < dt:
                self.Manipulator.apply_voltages(self.calculate_output_voltages())
                if ee_pos_values is not None:
                    ee_pos_values.append(self.Manipulator.get_ee_position())


    def Achieve_EE_Position(self, position_tup):
        # Acheives the given ee position
        if not self.Manipulator.LiesInWorkspace(position_tup):
            print("ERROR:: Given position out of workspace")
            return None
        self.graph_point = self.Manipulator.ax.scatter(position_tup[0], position_tup[1], position_tup[2])        
        q_mat = self.Manipulator.inv_kin(np.array(position_tup))
        self.set_target_state(np.array(np.array(q_mat).tolist() + 3*[0]))
        self.achieve_target_state()