import numpy as np;
import matplotlib.pyplot as pl;

def quat_norm(q):
	return np.sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3])

t			= np.loadtxt('t.csv', delimiter=',');
quat_th		= np.loadtxt('quaternion_th.csv', delimiter=',');
omega_th 	= np.loadtxt('omega_th.csv', delimiter=',');
quat_no		= np.loadtxt('quaternion_noise.csv', delimiter=',');
omega_no	= np.loadtxt('omega_noise.csv', delimiter=',');

quat_pred 	= np.loadtxt('quaternion_pred.csv', delimiter=',');
omega_pred 	= np.loadtxt('omega_pred.csv', delimiter=',');
delta_t		= np.loadtxt('delta_t.csv', delimiter=',');

# plot quat_predict and omega_predict
pl.subplots(2,4, figsize=(10,7))
pl.suptitle("Quaternion")
pl.subplot(2,2,1)
pl.plot(t, quat_th[0,:], label='Theoretical')
pl.plot(t, quat_pred[0,:], label='Predicted')
pl.plot(t, quat_no[0,:], label='Sent', linestyle='dashed')
pl.legend()
pl.xlabel("Time (s)")
pl.ylabel(r"$\eta$")
pl.grid(True)
pl.ylim(-1.1,1.1)
pl.legend()
pl.subplot(2,2,2)
pl.plot(t, quat_th[1,:], label='Theoretical')
pl.plot(t, quat_pred[1,:], label='Predicted')
pl.plot(t, quat_no[1,:], label='Sent', linestyle='dashed')
pl.xlabel("Time (s)")
pl.ylabel(r"$\epsilon_x$")
pl.grid(True)
pl.ylim(-1.1,1.1)
pl.legend()
pl.subplot(2,2,3)
pl.plot(t, quat_th[2,:], label='Theoretical')
pl.plot(t, quat_pred[2,:], label='Predicted')
pl.plot(t, quat_no[2,:], label='Sent', linestyle='dashed')
pl.xlabel("Time (s)")
pl.ylabel(r"$\epsilon_y$")
pl.grid(True)
pl.ylim(-1.1,1.1)
pl.legend()
pl.subplot(2,2,4)
pl.plot(t, quat_th[3,:], label='Theoretical')
pl.plot(t, quat_pred[3,:], label='Predicted')
pl.plot(t, quat_no[3,:], label='Sent', linestyle='dashed')
pl.xlabel("Time (s)")
pl.ylabel(r"$\epsilon_z$")
pl.grid(True)
pl.ylim(-1.1,1.1)
pl.legend()

pl.subplots(2,4, figsize=(10,7))
pl.suptitle("Angular rate and timing")
pl.subplot(2,2,1)
pl.plot(t, omega_th[1,:], label='Theoretical')
pl.plot(t, omega_pred[1,:], label='Predicted')
pl.plot(t, omega_no[1,:], label='Sent', linestyle='dashed')
pl.xlabel("Time (s)")
pl.ylabel(r"$\omega_x$")
pl.grid(True)
pl.legend()
pl.ylim(-2*np.pi, np.pi)
pl.subplot(2,2,2)
pl.plot(t, omega_th[2,:], label='Theoretical')
pl.plot(t, omega_pred[2,:], label='Predicted')
pl.plot(t, omega_no[2,:], label='Sent', linestyle='dashed')
pl.xlabel("Time (s)")
pl.ylabel(r"$\omega_y$")
pl.grid(True)
pl.legend()
pl.ylim(-2*np.pi, np.pi)
pl.subplot(2,2,3)
pl.plot(t, omega_th[3,:], label='Theoretical')
pl.plot(t, omega_pred[3,:], label='Predicted')
pl.plot(t, omega_no[3,:], label='Sent', linestyle='dashed')
pl.xlabel("Time (s)")
pl.ylabel(r"$\omega_z$")
pl.grid(True)
pl.legend()
pl.ylim(-2*np.pi, np.pi)
pl.subplot(2,2,4)
pl.plot(t, quat_norm(quat_th), label='Theoretical')
pl.plot(t, quat_norm(quat_pred), label='Predicted')
pl.plot(t, quat_norm(quat_no), label='Sent', linestyle='dashed')
pl.xlabel("Time (s)")
pl.ylabel("Norm of quat")
pl.ylim(0,2)
pl.grid(True)
pl.legend();

pl.show();
