import numpy as np
import matplotlib.pyplot as plt
import pdb
import math

numberAgents = 1
timeStep = 1.0
arenaLength = 2
shepherdSpeed = 2.0
maxAgentSpeed = 0.1
accuracy = 0.9999        # corresponds to (approx.) 0.81x2 = 1.62 degrees error
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
	agentPosition = np.random.uniform(low=-arenaLength/2, high=arenaLength/2, size=(2,1))
	shepherdPosition = np.random.uniform(low=-arenaLength/2, high=arenaLength/2, size=(2,1))

	# agentPosition = np.array([[0.08802829],[-0.67137385]])
	# shepherdPosition = np.array([[0.68631545],[-0.90820241]])
	print(agentPosition, shepherdPosition)
	shepherdVelocity = np.zeros((2,1), dtype=np.float32)
	agentPositionList = [agentPosition]
	shepherdPositionList = [shepherdPosition]

	v1 = shepherdPosition - destination
	v2 = agentPosition - destination
	dot = np.sum(v1*v2)
	det = v1[0,0]*v2[1,0] - v2[0,0]*v1[1,0]
	angle = math.atan2(det, dot)*180/np.pi
	isClockwise = -1 if angle > 0 else 1
	# pdb.set_trace()

	rotationMatrix = np.array([[0, isClockwise*1],[-isClockwise*1, 0]], dtype=np.float32)
	while(True):
		Xda = agentPosition - destination
		Xds = shepherdPosition - destination
		Xda = Xda/np.linalg.norm(Xda)
		Xds = Xds/np.linalg.norm(Xds)
		if(np.matmul(Xda.T, Xds) > accuracy):
			flag = True
			break
		lineOfSightVector = shepherdPosition - agentPosition
		dsa = np.linalg.norm(lineOfSightVector)
		lineOfSightVector = lineOfSightVector/dsa
		agentSpeed = calcSpeed(dsa)
		agentVelocity = -agentSpeed*lineOfSightVector
		shepherdVelocity = np.matmul(rotationMatrix, lineOfSightVector)*shepherdSpeed

		shepherdPosition = getUpdatedPosition(shepherdPosition, shepherdVelocity)
		agentPosition = getUpdatedPosition(agentPosition, agentVelocity)
		agentPositionList.append(agentPosition)
		shepherdPositionList.append(shepherdPosition)
		count+=1

	agentPositionList = np.array(agentPositionList)
	shepherdPositionList = np.array(shepherdPositionList)
	print(agentPositionList.shape)
	print("Number of Iterations : ", count)
	fig = plt.figure(1)
	plt.plot(destination[0,0],destination[1,0], marker='x')
	plt.plot(agentPositionList[:,0,0], agentPositionList[:,1,0], 'r')
	plt.plot(shepherdPositionList[:,0,0], shepherdPositionList[:,1,0], 'b')
	plt.plot([shepherdPositionList[-1,0,0],destination[0,0]],[shepherdPositionList[-1,1,0],destination[1,0]], 'g--')
	
	# plt.ylim((-arenaLength*factor, +arenaLength*factor))
	# plt.xlim((-arenaLength*factor, +arenaLength*factor))
	plt.grid(True)
	title = '../results-plots/oneAgentShepherdSpeed_'+str(shepherdSpeed)+'.png'
	fig.savefig(title)
	plt.show()

if __name__=='__main__':
	main()