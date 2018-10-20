import numpy as np 
import matplotlib.pyplot as plt 
import pdb
import warnings

warnings.simplefilter("error", RuntimeWarning)

numberAgents = 2
timeStep = 1.0
alpha = 2
k = 0.005
shepherdSpeed = 0.001
numberIteration = 100000

flag = False
sign = -1.0
prevSlope = 5

def getForce(distance):
	f = k/pow(distance, alpha)
	return f

def getUpdatedPosition(position, velocity):
	return position+velocity*timeStep

def main():
	global flag, sign
	destination = np.array([[5],[5]], dtype=np.float32)
	shepherdPosition = np.array([[0],[5]], dtype=np.float32)
	agentPosition = np.array([[3, 5],[7, 5]], dtype=np.float32)
	radius = np.sqrt(np.matmul(np.transpose(destination-shepherdPosition), (destination-shepherdPosition)))
	shepherdVelocity = np.zeros((2,1), dtype=np.float32)
	agentPositionList = [agentPosition]
	shepherdPositionList = [shepherdPosition]
	count = 0
	while(count < numberIteration):
		radius = np.sqrt(np.matmul(np.transpose(destination-shepherdPosition), (destination-shepherdPosition)))
		# print(radius)
		try:
			slope = (shepherdPosition[1,0]-destination[1,0])/(shepherdPosition[0,0]-destination[0,0])
			# print(slope)
		except RuntimeWarning:
			flag = True
		
		if (slope < 0 and prevSlope > 0):
			sign = -1*sign

		if(flag == False):
			shepherdVelocity[1,0] = sign*shepherdSpeed/np.sqrt(1+pow(slope,2))
			shepherdVelocity[0,0] =  -slope*shepherdVelocity[1,0]
		else:
			if(shepherdPosition[1,0] < destination[1,0]):
				shepherdVelocity = np.array([[shepherdSpeed],[0]], dtype=np.float32)
			else:
				shepherdVelocity = np.array([[-shepherdSpeed],[0]], dtype=np.float32)
			flag = False
		lineOfSightVector = agentPosition-shepherdPosition.T
		distance = np.reshape(np.linalg.norm(lineOfSightVector, axis=1), (2,1))
		force = getForce(distance)
		agentSpeed = force
		agentVelocity = agentSpeed*(lineOfSightVector/distance)
		# pdb.set_trace()
		#update step
		shepherdPosition = getUpdatedPosition(shepherdPosition, shepherdVelocity)
		agentPosition = getUpdatedPosition(agentPosition, agentVelocity)
		agentPositionList.append(agentPosition)
		shepherdPositionList.append(shepherdPosition)
		prevSlope = slope
		count+=1

	agentPositionList = np.array(agentPositionList)
	shepherdPositionList = np.array(shepherdPositionList)

	# plt.figure(1)
	# plt.subplot(211);plt.plot(agentPositionList[:,0,0], agentPositionList[:,0,1])
	# plt.subplot(212);plt.plot(agentPositionList[:,1,0], agentPositionList[:,1,1])
	# plt.show()

	# plt.figure(2)
	# plt.plot(shepherdPositionList[:,0,0], shepherdPositionList[:,1,0], 'r--')
	# plt.show()

	plt.figure(1)
	plt.plot(agentPositionList[:,0,0], agentPositionList[:,0,1], 'r--')
	plt.plot(agentPositionList[:,1,0], agentPositionList[:,1,1], 'g--')
	plt.plot(shepherdPositionList[:,0,0], shepherdPositionList[:,1,0], 'b--')
	plt.grid(True)
	plt.show()

if __name__=='__main__':
	main()