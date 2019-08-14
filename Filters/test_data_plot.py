import numpy as np
import matplotlib.pyplot as pl

def quat_norm(q):
	return np.sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3])

def quat_conj(q):
	res = np.zeros((4,1))
	res[0] =  q[0]
	res[1] = -q[1]
	res[2] = -q[2]
	res[3] = -q[3]
	return res

def quat_inv(q):
	res = np.zeros((4,1))
	res = quat_conj(q)/quat_norm(q)
	return res

def quat_mul(leftQ, rightQ):
	res = np.zeros((4,1))
	res[0] = leftQ[0]*rightQ[0] - leftQ[1]*rightQ[1] - leftQ[2]*rightQ[2] - leftQ[3]*rightQ[3]
	res[1] = leftQ[1]*rightQ[0] + leftQ[0]*rightQ[1] + leftQ[2]*rightQ[3] - leftQ[3]*rightQ[2]
	res[2] = leftQ[0]*rightQ[2] - leftQ[1]*rightQ[3] + leftQ[2]*rightQ[0] + leftQ[3]*rightQ[1]
	res[3] = leftQ[0]*rightQ[3] + leftQ[1]*rightQ[2] - leftQ[2]*rightQ[1] + leftQ[3]*rightQ[0]
	return res

def quat2euler(q):
	test = q[1]*q[2] + q[3]*q[0]
	if (test > 0.499):
		heading = 2 * np.arctan2(q[1],q[0])
		attitude = np.pi/2
		bank = 0
		return

	if (test < -0.499):
		heading = -2 * np.arctan2(q[1],q[0])
		attitude = - np.pi/2
		bank = 0
		return
	
	sqx = q[1]*q[1]
	sqy = q[2]*q[2]
	sqz = q[3]*q[3]
	heading = np.arctan2(2*q[2]*q[0]-2*q[1]*q[3] , 1 - 2*sqy - 2*sqz)
	attitude = np.arcsin(2*test)
	bank = np.arctan2(2*q[1]*q[0]-2*q[2]*q[3] , 1 - 2*sqx - 2*sqz)
	euler = np.zeros(3)
	euler[0] = bank
	euler[1] = attitude
	euler[2] = heading
	return euler

t			= np.loadtxt('t.csv', delimiter=',')
quat_th		= np.loadtxt('quaternion_th.csv', delimiter=',')
omega_th 	= np.loadtxt('omega_th.csv', delimiter=',')
quat_no		= np.loadtxt('quaternion_noise.csv', delimiter=',')
omega_no	= np.loadtxt('omega_noise.csv', delimiter=',')

quat_pred 	= np.loadtxt('quaternion_pred.csv', delimiter=',')
omega_pred 	= np.loadtxt('omega_pred.csv', delimiter=',')
delta_t		= np.loadtxt('delta_t.csv', delimiter=',')

filtered_error = np.zeros((3,t.size))
initial_error = np.zeros((3,t.size))
for i in range(t.size):
	filtered_error[:,i] = quat2euler( quat_mul( quat_th[:,i], quat_inv(quat_pred[:,i]) ) )
for i in range(t.size):
	initial_error[:,i] = quat2euler( quat_mul( quat_th[:,i], quat_inv(quat_no[:,i]) ) )

# plot quat_predict and omega_predict
pl.subplots(2,4, figsize=(10,7))
pl.suptitle("Quaternion")
pl.subplot(2,2,1)
pl.plot(t, quat_no[0,:], label='Noisy input', linestyle='none', marker="+", color='grey')
pl.plot(t, quat_th[0,:], label='Theoretical attitude', linestyle='dashed')
pl.plot(t, quat_pred[0,:], label='Filtered output')
pl.legend()
pl.xlabel("Time (s)")
pl.ylabel(r"$\eta$")
pl.grid(True)
pl.ylim(-1.1,1.1)
pl.legend()
pl.subplot(2,2,2)
pl.plot(t, quat_no[1,:], label='Noisy input', linestyle='none', marker="+", color='grey')
pl.plot(t, quat_th[1,:], label='Theoretical attitude', linestyle='dashed')
pl.plot(t, quat_pred[1,:], label='Filtered output')
pl.xlabel("Time (s)")
pl.ylabel(r"$\epsilon_x$")
pl.grid(True)
pl.ylim(-1.1,1.1)
pl.legend()
pl.subplot(2,2,3)
pl.plot(t, quat_no[2,:], label='Noisy input', linestyle='none', marker="+", color='grey')
pl.plot(t, quat_th[2,:], label='Theoretical attitude', linestyle='dashed')
pl.plot(t, quat_pred[2,:], label='Filtered output')
pl.xlabel("Time (s)")
pl.ylabel(r"$\epsilon_y$")
pl.grid(True)
pl.ylim(-1.1,1.1)
pl.legend()
pl.subplot(2,2,4)
pl.plot(t, quat_no[3,:], label='Noisy input', linestyle='none', marker="+", color='grey')
pl.plot(t, quat_th[3,:], label='Theoretical attitude', linestyle='dashed')
pl.plot(t, quat_pred[3,:], label='Filtered output')
pl.xlabel("Time (s)")
pl.ylabel(r"$\epsilon_z$")
pl.grid(True)
pl.ylim(-1.1,1.1)
pl.legend()

pl.subplots(3,1, figsize=(10,7))
pl.suptitle('Angular error between theoretical input and actual output')
pl.subplot(3,1,1)
pl.plot(t, initial_error[0]*180/np.pi, label="Input error", linestyle='dashed')
pl.plot(t, filtered_error[0]*180/np.pi, label="Output error")
pl.xlabel("Time (s)")
pl.ylabel("Roll anglular error (deg)")
pl.ylim(-10,10)
pl.grid(True)
pl.legend()
pl.subplot(3,1,2)
pl.plot(t, initial_error[1]*180/np.pi, label="Input error", linestyle='dashed')
pl.plot(t, filtered_error[1]*180/np.pi, label="Output error")
pl.xlabel("Time (s)")
pl.ylabel("Pitch anglular error (deg)")
pl.ylim(-10,10)
pl.grid(True)
pl.legend()
pl.subplot(3,1,3)
pl.plot(t, initial_error[2]*180/np.pi, label="Input error", linestyle='dashed')
pl.plot(t, filtered_error[2]*180/np.pi, label="Output error")
pl.xlabel("Time (s)")
pl.ylabel("Yaw anglular error (deg)")
pl.ylim(-10,10)
pl.grid(True)
pl.legend()

pl.subplots(2,4, figsize=(10,7),)
pl.suptitle("Angular rate and timing")
pl.subplot(2,2,1)
# pl.plot(t, omega_no[1,:], label='Measured', linestyle='dashed', color='grey')
pl.plot(t, omega_th[1,:], label='Theoretical rate', linestyle='dashed')
pl.plot(t, omega_pred[1,:], label='Filtered output')
pl.xlabel("Time (s)")
pl.ylabel(r"$\omega_x$")
pl.grid(True)
pl.legend()
pl.ylim(-2*np.pi, np.pi)
pl.subplot(2,2,2)
# pl.plot(t, omega_no[2,:], label='Measured', color='grey')
pl.plot(t, omega_th[2,:], label='Theoretical rate', linestyle='dashed')
pl.plot(t, omega_pred[2,:], label='Filtered output')
pl.xlabel("Time (s)")
pl.ylabel(r"$\omega_y$")
pl.grid(True)
pl.legend()
pl.ylim(-2*np.pi, np.pi)
pl.subplot(2,2,3)
# pl.plot(t, omega_no[3,:], label='Measured', linestyle='dashed', color='grey')
pl.plot(t, omega_th[3,:], label='Theoretical rate', linestyle='dashed')
pl.plot(t, omega_pred[3,:], label='Filtered output')
pl.xlabel("Time (s)")
pl.ylabel(r"$\omega_z$")
pl.grid(True)
pl.legend()
pl.ylim(-2*np.pi, np.pi)
pl.subplot(2,2,4)
pl.plot(t, delta_t/1000, label='Average {:f} ms'.format(np.average(delta_t)/1000))
pl.ylim(0,10)
pl.xlabel("Time (s)")
pl.ylabel("Timing (ms)")
pl.grid(True)
pl.legend()
for i in range(len(initial_error[0])):
	initial_error[0,i] = np.abs(np.max([initial_error[0,i],initial_error[1,i],initial_error[2,i]]))
	filtered_error[0,i] = np.abs(np.max([filtered_error[0,i],filtered_error[1,i],filtered_error[2,i]]))

print(np.max(filtered_error[0,20:])*180/np.pi)
print(np.min(filtered_error[0,20:])*180/np.pi)
print(np.average(filtered_error[0,20:])*180/np.pi)

pl.figure(figsize=(10,7))
pl.title('Angular error between theoretical input and actual output')
# pl.plot(t, initial_error[0]*180/np.pi, label="Input error", linestyle='dashed')
pl.plot(t, filtered_error[0]*180/np.pi, label="Output error")
pl.xlabel("Time (s)")
pl.ylabel("Maximum anglular error (deg)")
pl.ylim(0,10)
pl.grid(True)
pl.legend()

pl.show()
