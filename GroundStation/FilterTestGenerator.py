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
# Satellite inertia matrix
I_sat = np.diag(np.array([27, 17, 25], dtype="float"));

# # Handshake then send all data to board
# ser = serial.Serial('COM7', 115200, timeout=5);
# ready = False;
# while(not ready):
# 	if(ser.readline()):
# 		ready = True;
# 	else:
# 		print("waiting for board\n\r");
#
# ser.write(('{:f}\n\r'.format(sigma_p)).encode());
# ser.write(('{:f}\n\r'.format(sigma_q)).encode());
# ser.write(('{:f}\n\r'.format(sigma_eta)).encode());
# ser.write(('{:f}\n\r'.format(sigma_epsilon)).encode());
# ser.write(('{:f}\n\r'.format(sigma_omega)).encode());
#
# ser.write('{:f}\n\r'.format(I_sat[0,0]).encode());
# ser.write('{:f}\n\r'.format(I_sat[0,1]).encode());
# ser.write('{:f}\n\r'.format(I_sat[0,2]).encode());
# ser.write('{:f}\n\r'.format(I_sat[1,0]).encode());
# ser.write('{:f}\n\r'.format(I_sat[1,1]).encode());
# ser.write('{:f}\n\r'.format(I_sat[1,2]).encode());
# ser.write('{:f}\n\r'.format(I_sat[2,0]).encode());
# ser.write('{:f}\n\r'.format(I_sat[2,1]).encode());
# ser.write('{:f}\n\r'.format(I_sat[2,2]).encode());

# Generate omega_th(k)
omega_x = 2 * np.pi / 3;
omega_y = 2 * np.pi / 5;
omega_z = 2 * np.pi / 8;
t_end = 8;
N = 50;
t = np.linspace(0,t_end, N);
delta_t = np.zeros(t.size);
omega_th = np.zeros((4,t.size));
omega_th[1,:] = np.pi/4 * np.sin(omega_x * t);
omega_th[2,:] = np.pi/3 * np.sin(omega_y * t);
omega_th[3,:] = np.pi/2 * np.sin(omega_z * t);

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

# Saving for the board
quat_export = np.copy(quat_noise)
omega_export = np.copy(omega_noise[1:,:])
np.savetxt('../ADCS/Tests/quat.data', np.transpose(quat_export), fmt='%+f', delimiter=',', newline=',\n', header='size = {:d};\ndelta = {:f};\nfloat quat[{:d}] = {{'.format(N,t_end/N,4*t.size), footer='};', comments='')
np.savetxt('../ADCS/Tests/omega.data', np.transpose(omega_export), fmt='%+f', delimiter=',', newline=',\n', header='float omega[{:d}] = {{'.format(3*t.size), footer='};', comments='')

# Saving for the plotting
np.savetxt('t.csv', t, delimiter=',')
np.savetxt('quaternion_th.csv', quat_th, delimiter=',')
np.savetxt('omega_th.csv', omega_th, delimiter=',')
np.savetxt('quaternion_noise.csv', quat_noise, delimiter=',')
np.savetxt('omega_noise.csv', omega_noise, delimiter=',')

# Send quat and omega to board
# data = [0,0,0,0,0,0,0,0]
# for i in range(t.size):
# 	ser.write( ('{:f}\n\r'.format(quat_noise[0,i])).encode());
# 	ser.write( ('{:f}\n\r'.format(quat_noise[1,i])).encode());
# 	ser.write( ('{:f}\n\r'.format(quat_noise[2,i])).encode());
# 	ser.write( ('{:f}\n\r'.format(quat_noise[3,i])).encode());
# 	ser.write(('{:f}\n\r'.format(omega_noise[1,i])).encode());
# 	ser.write(('{:f}\n\r'.format(omega_noise[2,i])).encode());
# 	ser.write(('{:f}\n\r'.format(omega_noise[3,i])).encode());
# 	for i in range(8):
# 		data[i] = float(ser.readline());
# 	quat_pred[0,i]  = data[0]
# 	quat_pred[1,i]  = data[1]
# 	quat_pred[2,i]  = data[2]
# 	quat_pred[3,i]  = data[3]
# 	omega_pred[0,i] = data[4]
# 	omega_pred[1,i] = data[5]
# 	omega_pred[2,i] = data[6]
# 	delta_t[i]      = data[7]
