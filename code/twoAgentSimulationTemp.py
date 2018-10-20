import numpy as np
import matplotlib.pyplot as plt
import pdb
import math

numberAgents = 2
timeStep = 1.0
arenaLength = 2
shepherdSpeed = 0.1
maxAgentSpeed = 0.1
accuracy = 0.99       # corresponds to (approx.) 0.81x2 = 1.62 degrees error
flag = False
isClockwise = 1

def calcSpeed(distance): 
	speed = maxAgentSpeed*np.exp(-distance)
	return speed

def getUpdatedPosition(position, velocity):
	return position+velocity*timeStep

def main():
	count = 0
	global flag
	destination = np.zeros((2,1))
	agentPosition = np.random.uniform(low=-arenaLength/2, high=arenaLength/2, size=(2,2))
	shepherdPosition = np.random.uniform(low=-arenaLength/2, high=arenaLength/2, size=(2,1))
	centerOfMassPosition = np.reshape((agentPosition[:,0] + agentPosition[:,1])/2, (2, 1))

	shepherdVelocity = np.zeros((2,1), dtype=np.float32)
	agentVelocity = np.zeros((2,2), dtype=np.float32)
	agentPositionList = [agentPosition]
	shepherdPositionList = [shepherdPosition]

	v1 = shepherdPosition - destination
	v2 = centerOfMassPosition - destination
	dot = np.sum(v1*v2)
	det = v1[0,0]*v2[1,0] - v2[0,0]*v1[1,0]
	angle = math.atan2(det, dot)*180/np.pi
	isClockwise = -1 if angle > 0 else 1

	rotationMatrix = np.array([[0, isClockwise*1],[-isClockwise*1, 0]], dtype=np.float32)
	while(True):
		Xda = centerOfMassPosition - destination
		Xds = shepherdPosition - destination
		Xda = Xda/np.linalg.norm(Xda)
		Xds = Xds/np.linalg.norm(Xds)
		if(np.matmul(Xda.T, Xds) > accuracy):
			flag = True
			break
		lineOfSightVector = shepherdPosition - agentPosition
		dsa = np.linalg.norm(lineOfSightVector, axis=0)
		lineOfSightVector = lineOfSightVector/dsa
		agentSpeed = np.reshape(calcSpeed(dsa), (2,1))
		# pdb.set_trace()
		agentVelocity[:,0] = -lineOfSightVector[:,0]*agentSpeed[0,0]
		agentVelocity[:,1] = -lineOfSightVector[:,1]*agentSpeed[1,0]

		v = shepherdPosition - centerOfMassPosition
		shepherdVelocity = np.matmul(rotationMatrix, v/np.linalg.norm(v))*shepherdSpeed

		shepherdPosition = getUpdatedPosition(shepherdPosition, shepherdVelocity)
		agentPosition = getUpdatedPosition(agentPosition, agentVelocity)
		centerOfMassPosition = np.reshape((agentPosition[:,0] + agentPosition[:,1])/2, (2,1))
		agentPositionList.append(agentPosition)
		shepherdPositionList.append(shepherdPosition)
		count+=1

	agentPositionList = np.array(agentPositionList)
	shepherdPositionList = np.array(shepherdPositionList)
	print("Number of Iterations : ", count)
	plt.figure(1)
	plt.plot(destination[0,0], destination[1,0], marker='x')
	plt.plot(agentPositionList[:,0,0], agentPositionList[:,1,0], 'r', marker='.')
	plt.plot(agentPositionList[:,0,1], agentPositionList[:,1,1], 'g', marker='.')
	plt.plot(shepherdPositionList[:,0,0], shepherdPositionList[:,1,0], 'b')
	plt.plot([shepherdPositionList[-1,0,0],destination[0,0]],[shepherdPositionList[-1,1,0],destination[1,0]], 'g--')
	
	# plt.ylim((-arenaLength*factor, +arenaLength*factor))
	# plt.xlim((-arenaLength*factor, +arenaLength*factor))
	plt.grid(True)
	plt.show()

if __name__=='__main__':
	main()