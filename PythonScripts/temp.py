import numpy as np

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

q_th = np.array([[0.818337], [0.194846], [-0.396444],  [0.370385]])
q_es = np.array([[0.800018], [0.184047], [-0.409855], [0.397639]])

q_er = quat_mul(q_th,quat_inv(q_es))
print(quat2euler(q_er)*180/np.pi)