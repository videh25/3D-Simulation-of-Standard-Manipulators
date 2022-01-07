# Author: Videh Patel: videh.p@iitgn.ac.in
# Subsidiary file for the simulators to work

import numpy as np
import sympy as sp

def DHMatrix2Homo_and_Jacob(Hmat, prismatic=[]):
    # RETURN: Homogeneous Tranformation Matrix, Jacobian Matrix
    # Arguments:
            #Hmat: DH parameter matrix
                # DH Parameter Matrix [nx4 Matrix]
                # Number of links automatically calculated from the size of matrix
                # theta (rotation about z), d (translation about z), a(translation about x), alpha(rotation about x)
                # First columns of params are frame 0-->1 and last columns are frame (n-1)-->n :: n is the end effector frame
            #prismatic: An array of the joints that are prismatic: Joint corresponding to qi is ith joint
    # Note: All revolute and prismatic axes are assumed to be aligned with the respective z axis

    def d_(k):
        #Returns position vector of ee wrt k-frame in o-frame basis
        return (T[k][:3,:3])@((T[k]).inv()@T[n]@(sp.Matrix([[0],[0],[0],[1]])))[:3,0]
        
    n = len(Hmat)
    T = [sp.Matrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])]  #Array of all absolute Homogeneous Transformations : Has T0_0 at the start (Identity matrix)
    J = []  #Manipulator Jacobian
    for params in Hmat:
        theta, d, a, alpha = np.array(params)[0]
        Ti = sp.Matrix([ [ sp.cos(theta), -sp.sin(theta)*sp.cos(alpha), sp.sin(theta)*sp.sin(alpha), a*sp.cos(theta)],\
                         [ sp.sin(theta), sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha), a*sp.sin(theta)],\
                         [ 0            , sp.sin(alpha)               , sp.cos(alpha)              , d              ],\
                         [ 0            , 0                           , 0                          ,1              ] ])
        

        T.append(T[-1]@Ti)

    for i in range(n):
        if (i+1) in prismatic:
            Jv = T[i][:3,:3]@sp.Matrix([0,0,1])
            Jw = sp.Matrix([0,0,0])
        else: #Revolute
            Jw = T[i][:3,:3]@sp.Matrix([0,0,1])
            Jv = sp.Matrix(np.cross(Jw.T,d_(i).T).T)
        
        Ji = np.concatenate((sp.simplify(sp.nsimplify(sp.Matrix(Jv), [sp.pi], tolerance = 1e-10, rational= False)), sp.simplify(sp.nsimplify(sp.Matrix(Jw), [sp.pi], tolerance = 1e-10, rational= False))))
        
        if len(J) == 0:
            J = Ji
        else:
            J = np.concatenate((J,Ji), axis = 1)

        H = T[n]

    return H, J

def CG_from_DV(D,V):
    # Return: Christoffel Matrix, Potential feild matrix
    # Arguments:
        # D: Inertia Matrix
            # D(q)--> nxn sp.Matrix :: D should be in terms of q1,q2,q3... qn
        # V: Potential Feild
            # V(q)--> scalar potential :: D should be in terms of q1,q2,q3... qn

    q = []
    q_dot = []
    q_dot2 = []
    T = []
    n = sp.shape(D)[0]
    C = sp.zeros(n) #Christoffel symbol sp.Matrix
    g_ = sp.zeros(n,1) #Potential Field sp.Matrix
    for i in range(1,n+1):
        #Generating q sp.Matrix
        q_ = sp.symbols('q' + str(i))
        q.append(q_)

        qdot_ = sp.symbols('q' + str(i) + '_dot')
        q_dot.append(qdot_)

        qdot2_ = sp.symbols('q' + str(i) + '_dot2')
        q_dot2.append(qdot2_)

        tau_ = sp.symbols('tau' + str(i))
        T.append(tau_)



    for k in range(1,n+1):
        for j in range(1,n+1):
            c=0
            for i in range(1,n+1):
                c += 1/2*(sp.diff(D[k-1,j-1], q[i-1]) + sp.diff(D[k-1,i-1], q[j-1]) - sp.diff(D[i-1,j-1], q[k-1]))*q_dot[i-1]
            C[k-1,j-1] = c

            print('...Calculating C(i,j): ' + str((k,j)))


    for i in range(1,n+1):
        g_[i-1,0] = sp.diff(V, q[i-1])
        print('Calculating g_(i): ' + str(i))

    
    # for i,eqn in enumerate(D@sp.Matrix(q_dot2) + C@sp.Matrix(q_dot) + g_ - sp.Matrix(T)):
    #     print('Equation' + str(i+1) + '_______________________________________________-')
    #     print(str(eqn) + " = 0")
    #     print()

    return C, g_,

def DV_Calculator(Dhmi, g_dir ='-z', prismatic = []):
    # Return: D (Inertia Matrix), V (Potential Feild)
    # Arguments: 
        # Dhmi: Dhmi (DH + Mass + Inetria Tensor) array
            # [[theta1,      d1,     l1,     alpha1,    m1,     I1],
            #  [theta2,      d2,     l2,     alpha2,    m2,     I2]]
            # I1, I2 are 3x3 inertia tensors wrt the position of next link frame 
        # g_dir: Direction of gravity
            # Can be 'x', 'y', 'z', '-x', '-y', '-z'
            #prismatic: An array of the joints that are prismatic: Joint corresponding to qi is ith joint

    # Note:
        #Assumes the centres of mass of links to lie at the middle
        #A link is assumed with given mass connecting each frame
        #Also, assumes the links to have moment of inertia of (m*l**2)/12
        #mass_arr is symbolic matrix
        #Dhmi is a matrix with manipulator lengths, link masses passed as constants and joint variables passed as symbolic constants
        #g_dir is used to calculate the V, specifies the direction of g; Default is taken as -z
        #Can also pass everything as symbolic constants
        
    n = sp.shape(Dhmi)[0]
    D = np.matrix(np.zeros((n,n)))
    V = 0

    dir_check_index = 0
    if g_dir[0] == '-':
        dir_m = 1
        dir_check_index = 1
    else:
        dir_m = -1

    if g_dir[dir_check_index] == 'z':
        dir_i = 2
    elif g_dir[dir_check_index] == 'y':
        dir_i = 1
    elif g_dir[dir_check_index] == 'x':
        dir_i = 0

    print('Starting to calculate Inertia matrix(D) and potential function(V)')
    for i in range(1, n+1):
        print('... Iterating for D & V:: loop: ' + str(i))
        Dh_ = np.concatenate((Dhmi[:i-1,:4], np.matrix([Dhmi[i-1,0], Dhmi[i-1,1]/2, Dhmi[i-1,2]/2, Dhmi[i-1,3]])))
        H, J = DHMatrix2Homo_and_Jacob(Dh_, [x for x in prismatic if x<=i])

        Jv = sp.nsimplify(sp.Matrix(J[:3,:]).row_join(sp.zeros(3,n-i)), constants = [sp.pi], tolerance = 1e-5, rational = False)
        Jw = sp.nsimplify(sp.Matrix(J[3:,:]).row_join(sp.zeros(3,n-i)), constants = [sp.pi],tolerance = 1e-5, rational = False)
        R = sp.simplify(H[:3,:3])

        I = Dhmi[i-1,5]
        m = Dhmi[i-1,4]
        g = sp.symbols('g')

        D = D + sp.simplify(m*Jv.T@Jv + Jw.T@R@I@R.T@Jw)
        V = V + sp.simplify(dir_m*m*g*H[dir_i,3])
    
    return sp.simplify(sp.nsimplify(D, tolerance = 1e-10)), sp.simplify(sp.nsimplify(V, tolerance = 1e-10))

def Dynamic_Equations_from_Dhmi(Dhmi, g_dir ='-z', prismatic = []):
    #Calculates the dynamic equations using the Dhmni and prismatic matrix passed
    
    Dexp, Vexp = DV_Calculator(Dhmi, g_dir = g_dir, prismatic = prismatic)
    
    C, g_ = CG_from_DV(Dexp,Vexp)
    print('Calculated C and g_')
    return Dexp, sp.simplify(sp.nsimplify(C, tolerance = 1e-10)), sp.simplify(sp.nsimplify(g_, tolerance = 1e-10))

def q_dot2_array(Dhmi, tau, q_dot, g_dir ='-z', prismatic = []):
    # Returns array of q_dot2 for the given symbolic matrix of Dhmi tau and q_dot
    Dexp, C, g_ = Dynamic_Equations_from_Dhmi(Dhmi, g_dir = g_dir, prismatic = prismatic)

    return Dexp.inv()@(tau - C@q_dot - g_)
