# -*- coding: utf-8 -*-
"""
Created on Sun May 28 04:18:24 2023

@author: elsecredit
"""

import numpy as np

# STANCE PHASE

# Data

cmd_f = -0.1406
cmd_c = 0.2580
cmd_t = 0.2570

ang_a1 = (np.pi/2) - 0.001
ang_k1 = 0.072
ang_h1 = (np.pi/2) - 0.378

long_f = 0.2812
long_c = 0.4551
long_t = 0.4533

    # Vectors
vect_pos = np.array([0, 0, 0, 1])
vect_calf = np.array([long_c, 0, 0, 1])
vect_tight = np.array([long_t, 0, 0, 1])

# Positions 

    # Joint Positions

        # Position ankle   
print('Position ankle stance')
Pa = np.array([0, 0, 0, 1])

        # Position knee
Ra = np.array([[np.cos(ang_a1), -np.sin(ang_a1), 0, 0],
               [np.sin(ang_a1), np.cos(ang_a1), 0, 0],
               [0, 0, 1, 0],
               [0, 0, 0, 1]])  # Rotation ankle
RTk = np.array([[np.cos(ang_k1), -np.sin(ang_k1), 0, long_c],
                [np.sin(ang_k1), np.cos(ang_k1), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]])  # Rototranslation knee
print('Position knee stance')
Pk = np.dot(Ra, vect_calf)

        # Position hip
print('Position hip stance')
Ph = np.dot(np.dot(Ra, RTk), vect_tight)


    # CG Positions
    
        # CG foot
print('CG foot stance')
PCGf = np.array([cmd_f, 0, 0, 1])

        # CG calf
v_cmdc = np.array([cmd_c, 0, 0, 1])
print('CG calf stance')
PCGc = np.dot(Ra, v_cmdc)

        # CG tight
v_cmdt = np.array([cmd_t, 0, 0, 1])
print('CG tight stance')
PCGt = np.dot(Ra, np.dot(RTk, v_cmdt))


# Accelerations

PCGc = PCGc[:3]
Pk = Pk[:3]
PCGt = PCGt[:3]

    # Angular velocities 
w_a = np.array([0, 0, 0.2066])
w_k = np.array([0, 0, -0.7218])
w_h = np.array([0, 0, -1.4954])

    # Angular accelerations 
alpha_a = np.array([0, 0, -10.871])
alpha_k = np.array([0, 0, 2.298])


    # CG Accelerations
A_a = 0  # Assume linear acceleration of the ankle as zero

        # Foot
print('Acceleration foot stance')
A_f = 0

        # Calf
print('Acceleration calf stance')
A_c = A_a + np.cross(w_a, np.cross(w_a, PCGc)) + np.cross(alpha_a, PCGc)

        # Knee
print('Acceleration knee stance')
A_k = A_a + np.cross(w_a, np.cross(w_a, Pk)) + np.cross(alpha_a, Pk)

        # Tight:
print('Acceleration tight stance')
A_t = A_k + np.cross((w_k + w_a),np.cross((w_k + w_a), (PCGt-Pk)) + np.cross((alpha_k + alpha_a), (PCGt-Pk)))


# Forces and moments

    # Ground reaction forces 
F_ve = 70.7923  # Vertical
F_ap = -4.1752  # Horizontal 

    # Data
mTB = 85
dTH = 1.85
m_c = 3.9525
m_t = 8.5

W_f = 1.2325 * 9.81
W_c = m_c * 9.81
W_t = m_t * 9.81

cmd_c = 0.2580

I_c = 0.0746
I_t = 0.1822

alpha_a = alpha_a[2, :]
alpha_k = alpha_k[2, :]

    # We find components x and y components of cmp and cmd
        # Foot
cmp_f = 0.1406

        # Calf
cmp_c_v = Pk - PCGc

cmp_cx = cmp_c_v[0, :]
cmp_cy = cmp_c_v[1, :]

Pa = Pa[:3, :]
cmd_c_v = PCGc - Pa

cmd_cx = cmd_c_v[0, :]
cmd_cy = cmd_c_v[1, :]

        # Tight
cmp_t_v = PCGt - Pk

cmp_tx = cmp_t_v[0, :]
cmp_ty = cmp_t_v[1, :]

Ph = Ph[:3, :]
cmd_t_v = Ph - PCGt

cmd_tx = cmd_t_v[0, :]
cmd_ty = cmd_t_v[1, :]


    # We fins x and y components of acceleration
A_cx = A_c[0, :]
A_cy = A_c[1, :]

A_tx = A_t[0, :]
A_ty = A_t[1, :]

    # ANKLE JOINT
        # Ankle joint x axis
F_ax = -F_ap

        # Ankle joint y axis
F_ay = W_f - F_ve

        # Moment ankle
M_a = W_f * cmp_f + F_ve * cmp_f

        # Moment normalitzat
M_a_n = M_a / (mTB * dTH)

    # KNEE JOINT
        # Knee joint x axis
F_kx = F_ax - m_c * A_cx

        # Knee joint normalized x axis
F_kx_n = F_kx / mTB

        # Knee joint y axis
F_ky = -W_c - F_ay - m_c * A_cy

        # Knee joint y axis normalitzat
F_ky_n = F_ky / mTB

        # Moment knee
M_k = M_a + F_ay * cmd_cx - F_ky * cmp_cx + F_ax * cmd_cy + F_kx * cmp_cy - I_c * alpha_a

        # Normalized moment
M_k_n = M_k/(mTB*dTH)

    # HIP JOINT
        # Hip joint x axis
F_hx = F_kx - m_t * A_tx

        # Hip joint normalized x axis
F_hx_n = F_hx / mTB

        # Hip joint y axis
F_hy = -W_t + F_ky - m_t * A_ty

        # Hip joint normalized y axis
F_hy_n = F_hy / mTB

        # Moment hip
M_h = M_k - F_ky * cmd_tx + F_kx * cmd_ty - F_hy * cmp_tx + F_hx * cmp_ty - I_t * alpha_k

        # Normalized moment hip
M_h_n = M_h / (mTB * dTH)


# Potència

    # ANKLE JOINT
Pw_a = M_a * w_a
Pw_a = Pw_a[2]

    # KNEE JOINT
Pw_k = M_k * w_k
Pw_k = Pw_k[2]

    # HIP JOINT
Pw_h = M_h * w_h
Pw_h = Pw_h[2]       


# SWING PHASE

# Data

ang_h2 = (np.pi/2) - 0.218;
ang_k2 = 0.709;
ang_a2 = -((np.pi/2) + 0.222); 

cmp_f = -0.1406;
cmp_c = -0.1971;
cmp_t = -0.1963;

long_f = -0.2812;
long_c = -0.4551;
long_t = -0.4533;

    # Vectors
vect_pos = np.array([0, 0, 0, 1])
vect_calf = np.array([long_c, 0, 0, 1])
vect_tight = np.array([long_t, 0, 0, 1])

# Positions 

    # Joint Positions

        # Position hip   
print('Position hip swing')
Ph2 = np.array([0, 0, 0, 1])

        # Position knee
Rh2 = np.array([[np.cos(ang_h2), -np.sin(ang_h2), 0, 0],
               [np.sin(ang_h2), np.cos(ang_h2), 0, 0],
               [0, 0, 1, 0],
               [0, 0, 0, 1]])  # Rotacion hip
RTk2 = np.array([[np.cos(ang_k2), -np.sin(ang_k2), 0, long_t],
                [np.sin(ang_k2), np.cos(ang_k2), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]])  # Rototranslation knee
print('Position knee swing')
Pk2 = np.dot(Ra, vect_calf)

        # Position ankle
RTa2 = np.array([[np.cos(ang_a2), -np.sin(ang_a2), 0, long_c],
                [np.sin(ang_a2), np.cos(ang_a2), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]])  # Rototranslation ankle
print('Position ankle swing')
Pa2 = np.dot(np.dot(Rh2, RTk2), vect_calf)


    # CG Positions
    
        # CG tigh    
v_cmpt2 = np.array([cmp_t, 0, 0, 1]) 
print('CG tight swing')
PCGt2 = np.dot(Rh2, v_cmpt2)

        # CG calf
v_cmpc2 = np.array([cmp_c, 0, 0, 1]) 
print('CG calf swing')
PCGc2 = np.dot(np.dot(Rh2, RTk2), v_cmpc2) 

        # CG foot
v_cmpf2 = np.array([cmp_f, 0, 0, 1]) 
print('CG foot swing')
PCGf2 = np.dot(np.dot(np.dot(Rh2, RTk2), RTa2), v_cmpf2)


# Accelerations

PCGt2 = PCGt2[:3]
PCGc2 = PCGc2[:3]
PCGf2 = PCGf2[:3]

Ph2 = Ph2[:3, :]
Pk2 = Pk2[:3, :]
Pa2 = Pa2[:3, :]

    # Angular velocities 
w_h = np.array([0, 0, 3.0946])
w_k = np.array([0, 0, 5.4155])
w_a = np.array([0, 0, -0.9284])

    # Angular accelerations 
alpha_h = np.array([0, 0, 26.268])
alpha_k = np.array([0, 0, -18.312])
alpha_a = np.array([0, 0, 86.863])


    # CG Accelerations
A_h = 0  # Assume linear acceleration of the hip as zero

        # Tight
print('Acceleration tight swing')
A_t = A_h + np.cross(w_h, np.cross(w_h, PCGt2)) + np.cross(alpha_h, PCGt2)  # assumim acceleració linial del hip com a zero
print(A_t)

        # Knee
print('Acceleration knee swing')
A_k = A_h + np.cross(w_h, np.cross(w_h, Pk2)) + np.cross(alpha_h, Pk2)
print(A_k)

        # Calf
print('Acceleration calf swing')
A_c = A_k + np.cross((w_k + w_h), np.cross((w_k + w_h), (PCGc2 - Pk2)) + np.cross((alpha_k + alpha_h), (PCGc2 - Pk2)))
print(A_c)

        # Ankle
print('Acceleration ankle swing')
A_a = A_k + np.cross((w_k + w_h), np.cross((w_k + w_h), (Pa2 - Pk2)) + np.cross((alpha_k + alpha_h), (Pa2 - Pk2)))
print(A_a)

        # Foot
print('Acceleration foot swing')
A_f = A_a + np.cross((w_h + w_k + w_a), np.cross((w_h + w_k + w_a), (PCGf2 - Pa2)) + np.cross((alpha_h + alpha_k + alpha_a), (PCGf2 - Pa2)))
print(A_f)       

# Forces and moments



    # Data
dTH = 1.85
mTB = 85
m_c = 3.9525
m_t = 8.5
m_f = 1.2325

W_f = 1.2325 * 9.81
W_c = m_c * 9.81
W_t = m_t * 9.81

cmd_c = 0.2580

I_c = 0.0746
I_t = 0.1822
I_f = 0.0220

alpha_a = alpha_a[2, :]
alpha_k = alpha_k[2, :]
alpha_h = alpha_h[2, :]


    # We find components x and y components of cmp and cmd
        # Tight
cmp_t_v = PCGt2 - Ph2  # Proximals

cmp_tx = cmp_t_v[0, :]
cmp_ty = cmp_t_v[1, :]

cmd_t_v = Pk2 - PCGt2  # Distals

cmd_tx = cmd_t_v[0, :]
cmd_ty = cmd_t_v[1, :]


        # Calf
cmp_c_v = PCGc2 - Pk2  # proximals

cmp_cx = cmp_c_v[0, :]
cmp_cy = cmp_c_v[1, :]

cmd_c_v = Pa2 - PCGc2  # distals

cmd_cx = cmd_c_v[0, :]
cmd_cy = cmd_c_v[1, :]

        # Foot
cmp_f_v = PCGf2 - Pa2  # proximals

cmp_fx = cmp_f_v[0, :]
cmp_fy = cmp_f_v[1, :]


    # We fins x and y components of acceleration
A_cx = A_c[0, :]
A_cy = A_c[1, :]

A_tx = A_t[0, :]
A_ty = A_t[1, :]

A_fx = A_f[0, :]
A_fy = A_f[1, :]

    # ANKLE JOINT
        # Ankle joint x axis
F_ax = -(m_f * A_fx)

        # Ankle joint normalized x axis
F_ax_n = F_ax/mTB

        # Ankle joint y axis
F_ay = m_f *A_fy + W_f

        # Ankle joint normalized y axis
F_ay_n = F_ay/mTB

        # Moment ankle
M_a = -F_ax * cmp_fy - F_ay * cmp_fx + I_f * alpha_a

        # Moment normalitzat
M_a_n = M_a/(mTB*dTH)

    # KNEE JOINT
        # Knee joint x axis
F_kx = -m_c * A_cx + F_ax

        # Knee joint normalized x axis
F_kx_n = F_kx/mTB

        # Knee joint y axis
F_ky = m_c *A_cy + W_c + F_ay

        # Knee joint y axis normalitzat
F_ky_n = F_ky/mTB

        # Moment knee
M_k = M_a + F_kx * cmp_cy - F_ky * cmp_cx + F_ax * cmd_cy - F_ay * cmd_cx + I_c * alpha_k

        # Normalized moment
M_k_n = M_k/(mTB*dTH)

    # HIP JOINT
        # Hip joint x axis
F_hx = m_t * A_tx - F_kx

        # Hip joint normalized x axis
F_hx_n = F_hx/mTB

        # Hip joint y axis
F_hy = m_t *A_ty + W_t + F_ky

        # Hip joint normalized y axis
F_hy_n = F_hy/mTB

        # Moment hip
M_h = M_k + F_hx * cmp_ty - F_hy * cmp_tx - F_kx * cmd_ty - F_ky * cmd_tx + I_t * alpha_h

        # Normalized moment hip
M_h_n = M_h/(mTB*dTH)


# Potència

    # ANKLE JOINT
Pw_a = M_a * w_a;
Pw_a = Pw_a[2]

    # KNEE JOINT
Pw_k = M_k * w_k;
Pw_k = Pw_k[2]

    # HIP JOINT
Pw_h = M_h * w_h;
Pw_h = Pw_h[2]     























