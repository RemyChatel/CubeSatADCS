import numpy as np
import matplotlib.pyplot as pl
import scipy.integrate as int

def quat_mul(leftQ, rightQ):
	res = np.zeros((4,1))
	res[0] = leftQ[0]*rightQ[0] - leftQ[1]*rightQ[1] - leftQ[2]*rightQ[2] - leftQ[3]*rightQ[3]
	res[1] = leftQ[1]*rightQ[0] + leftQ[0]*rightQ[1] + leftQ[2]*rightQ[3] - leftQ[3]*rightQ[2]
	res[2] = leftQ[0]*rightQ[2] - leftQ[1]*rightQ[3] + leftQ[2]*rightQ[0] + leftQ[3]*rightQ[1]
	res[3] = leftQ[0]*rightQ[3] + leftQ[1]*rightQ[2] - leftQ[2]*rightQ[1] + leftQ[3]*rightQ[0]
	return res

def quat_norm(q):
	return np.sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3])

def quat_exp(q):
	w = q[0]
	v = q[1:]
	tmp = np.zeros((4,1))
	tmp[0] = np.cos(quat_norm(v))
	tmp[1:] = v / quat_norm(v) * np.sin(quat_norm(v))
	return tmp * np.exp(w)


################################################################################
# Simulation parameters
t_end = 10
N = 1000
# Init quat_th0# Generate omega_th(k)
# ZYX 60->-45->30	    eta     x       y      z
# quat_th0  = np.array([0.0723,0.392,-0.2010, 0.5320], dtype="float")
quat_th0  = np.array([1,0,0,0], dtype="float")
omega_th0 = np.array([90 * np.pi/180, 45 * np.pi/180, 0 * np.pi/180], dtype="float")
# Kalman P initial covaraince matrix setup
sigma_p = 0.01
# Kalman Q process noise matrix setup
sigma_q = 0.01
# Kalman R measurement noise matrix setup
sigma_eta = 0.05
sigma_epsilon = 0.05
sigma_omega = 0.05

I = np.diag([27,17,25])
I_inv = np.linalg.inv(I)


t = np.linspace(0,t_end, N)
delta_t = np.zeros(t.size)
omega_th = np.zeros((4,t.size))
quat_th = np.zeros((4,t.size))
state = np.zeros((7,t.size))
state0 = np.zeros(7)
state0[:4] = quat_th0
state0[4:] = omega_th0

################################################################################
# INTEGRATION ROUTINE HERE
def int_op(x, t):
	tmp = np.zeros(7)
	quat = x[:4]
	omega = x[4:]
	tmp[0]   = -0.5*np.dot(quat[1:],omega)
	tmp[1:4] =  0.5*( quat[0]*omega + np.cross(quat[1:], omega) )
	tmp[4:]  = - np.matmul(I_inv, np.cross(omega, np.matmul(I, omega)))
	return tmp

state = int.odeint(int_op, state0, t)
state = np.transpose(state)
quat_th  = state[:4,:]
omega_th[1:,:] = state[4:,:]

for i in range(t.size):
	quat_th[:,i] /= np.linalg.norm(quat_th[:,i])


################################################################################
# Add noise and pack to state vec

quat_noise = np.zeros((4,t.size), dtype="float")
omega_noise = np.zeros((4,t.size), dtype="float")

quat_noise[0,:] = quat_th[0,:] + np.random.normal(0, sigma_eta, (t.size))
quat_noise[1:,:] = quat_th[1:,:] + np.random.normal(0, sigma_epsilon, (3,t.size))
omega_noise[1:,:] = omega_th[1:,:] + np.random.normal(0, sigma_omega, (3,t.size))
quat_pred = np.copy(quat_noise)
omega_pred = np.copy(omega_noise)


# Saving for the board
quat_export = np.copy(quat_noise)
omega_export = np.copy(omega_noise[1:,:])
string = ""
string += 'sigma_p = {:f};\n'.format(sigma_p)
string += 'sigma_q = {:f};\n'.format(sigma_q)
string += 'sigma_eta = {:f};\n'.format(sigma_eta)
string += 'sigma_epsilon = {:f};\n'.format(sigma_epsilon)
string += 'sigma_omega = {:f};\n'.format(sigma_omega)
string += 'size = {:d};\n'.format(N)
string += 'delta = {:f};\n'.format(t_end/N)
string += 'float quat[{:d}] = {{\n'.format(4*t.size)
np.savetxt('quat.data',np.transpose(quat_export), fmt='%+f', delimiter=',', newline=',\n', header=string, footer='};', comments='')
np.savetxt('omega.data',np.transpose(omega_export), fmt='%+f', delimiter=',', newline=',\n', header='float omega[{:d}] = {{\n'.format(3*t.size), footer='};', comments='')

# Saving for the plotting
np.savetxt('t.csv', t, delimiter=',')
np.savetxt('quaternion_th.csv', quat_th, delimiter=',')
np.savetxt('omega_th.csv', omega_th, delimiter=',')
np.savetxt('quaternion_noise.csv', quat_noise, delimiter=',')
np.savetxt('omega_noise.csv', omega_noise, delimiter=',')
