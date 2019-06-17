import numpy as np;
import matplotlib.pyplot as pl;

def quat_mul(leftQ, rightQ):
	res = np.zeros(4);
	a = leftQ[0]
	b = leftQ[1]
	c = leftQ[2]
	d = leftQ[3]
	e = rightQ[0]
	f = rightQ[1]
	g = rightQ[2]
	h = rightQ[3]
	res[0] = a*e - b*f - c*g- d*h
	res[1] = b*e + a*f + c*h - d*g
	res[2] = a*g - b*h + c*e + d*f
	res[3] = a*h + b*g - c*f + d*e
	return res

# Init quat_th0
# ZYX 60->-45->30	   eta     x       y      z
quat_th0 = np.array([0.0723, 0.392,-0.2010, 0.5320], dtype="float");
quat_th0 = np.array([0.0000, 1, 0.0000, 0.0000], dtype="float");
# Kalman P initial covaraince matrix setup
sigma_p = 0.1;
# Kalman Q process noise matrix setup
sigma_q = 0.001;
# Kalman R measurement noise matrix setup
sigma_eta = 0.01;
sigma_epsilon = 0.01;
sigma_omega = 0.1;

# Generate omega_th(k)
omega_x = 2 * np.pi / 3;
omega_y = 2 * np.pi / 5;
omega_z = 2 * np.pi / 8;
t_end = 8;
N = 100;
t = np.linspace(0,t_end, N);
delta_t = np.zeros(t.size);
omega_th = np.zeros((4,t.size));
omega_th[1,:] = 50 * np.pi/180 * np.sin(omega_x * t);
omega_th[2,:] = 30 * np.pi/180 * np.sin(omega_y * t);
omega_th[3,:] = 90 * np.pi/180 * np.sin(omega_z * t);

# Propagate to quat_th(k)
quat_th = np.zeros((4,t.size));

# INTEGRATION ROUTINE HERE
quat_th[:,0] = quat_th0;
for i in range(1,t.size):
	dt = t[i]-t[i-1]
	quat_old  = quat_th[:,i-1]
	omega_old = omega_th[:,i-1]
	integrande = quat_mul(quat_old, omega_old)
	quat_th[:,i] = quat_th[:,i-1] + dt * 0.5 * integrande
	quat_th[:,i] /= np.linalg.norm(quat_th[:,i])

# Add noise and pack to state vec
quat_noise = np.zeros((4,t.size), dtype="float");
omega_noise = np.zeros((4,t.size), dtype="float");
quat_noise[0,:] = quat_th[0,:] + np.random.normal(0, sigma_eta, (t.size));
quat_noise[1:,:] = quat_th[1:,:] + np.random.normal(0, sigma_epsilon, (3,t.size));
omega_noise[1:,:] = omega_th[1:,:] + np.random.normal(0, sigma_omega, (3,t.size));
quat_pred = np.copy(quat_noise)
omega_pred = np.copy(omega_noise)

for i in range(1,t.size):
	quat_noise[:,i] /= np.linalg.norm(quat_noise[:,i])

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
string += 'float quat[{:d}] = {{'.format(4*t.size)
np.savetxt('../ADCS/Tests/quat.data', np.transpose(quat_export), fmt='%+f', delimiter=',', newline=',\n', header=string, footer='};', comments='')
np.savetxt('../ADCS/Tests/omega.data', np.transpose(omega_export), fmt='%+f', delimiter=',', newline=',\n', header='float omega[{:d}] = {{'.format(3*t.size), footer='};', comments='')

# Saving for the plotting
np.savetxt('t.csv', t, delimiter=',')
np.savetxt('quaternion_th.csv', quat_th, delimiter=',')
np.savetxt('omega_th.csv', omega_th, delimiter=',')
np.savetxt('quaternion_noise.csv', quat_noise, delimiter=',')
np.savetxt('omega_noise.csv', omega_noise, delimiter=',')
