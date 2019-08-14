import numpy as np
import sys, serial
from time import sleep

try:
	ser = serial.Serial('COM8', 115200, timeout=10)
except KeyboardInterrupt:
	print('exiting')

sleep(0.1)

t = np.loadtxt('t.csv', delimiter=',')
size = t.size

quat_pred	= np.zeros((4,size), dtype='float')
omega_pred	= np.zeros((4,size), dtype='float')
delta_t 	= np.zeros((  size), dtype='float')

for i in range(size):
	try:
		line = ser.readline()
		print(line)
		data = [float(val) for val in line.split()]
		quat_pred[0,i] 	= data[0]
		quat_pred[1,i] 	= data[1]
		quat_pred[2,i] 	= data[2]
		quat_pred[3,i] 	= data[3]
		omega_pred[1:,i]= data[4]
		omega_pred[2:,i]= data[5]
		omega_pred[3:,i]= data[6]
		delta_t[i] 		= data[7]
	except KeyboardInterrupt:
		print('exiting')

ser.flush()
ser.close()

np.savetxt('quaternion_pred.csv', quat_pred, delimiter=',', fmt='%+f')
np.savetxt('omega_pred.csv', omega_pred, delimiter=',', fmt='%+f')
np.savetxt('delta_t.csv', delta_t, delimiter=',', fmt='%+f')


t			= np.loadtxt('t.csv', delimiter=',')
quat_th		= np.loadtxt('quaternion_th.csv', delimiter=',')
omega_th 	= np.loadtxt('omega_th.csv', delimiter=',')

quat_pred 	= np.loadtxt('quaternion_pred.csv', delimiter=',')
omega_pred 	= np.loadtxt('omega_pred.csv', delimiter=',')
delta_t		= np.loadtxt('delta_t.csv', delimiter=',')

exec(open("test_data_plot.py.py").read())
