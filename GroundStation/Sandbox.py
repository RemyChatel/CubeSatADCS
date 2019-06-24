import numpy as np
import matplotlib.pyplot as pl

t_end = 8;
N = 100;
t = np.linspace(0,t_end, N);
delta_t = np.zeros(t.size);
omega_th = np.zeros((4,t.size));
omega_th[1,:] = 50 * np.pi/180 # * np.sin(omega_x * t);
omega_th[2,:] = 30 * np.pi/180 # * np.sin(omega_y * t);
omega_th[3,:] = 90 * np.pi/180 # * np.sin(omega_z * t);
quat_th0 = np.array([[1.0000],[0.000],[ 0.0000], [0.0000]], dtype="float");

i = 0

A = np.zeros((4,4))
A[0,1] = -omega_th[1,i]
A[0,2] = -omega_th[2,i]
A[0,3] = -omega_th[3,i]
A[1,2] =  omega_th[3,i]
A[1,3] = -omega_th[2,i]
A[2,3] =  omega_th[1,i]
A -= np.transpose(A)

print(np.matmul(A, quat_th0))
