import numpy as np 
import matplotlib.pyplot as plt 
import random, math
import pdb

timeStep = 1.0
arenaLength = 2
numberAgents = 10

maxAgentSpeed = 0.1
expScaleFactor = 0.35   # plot tells that 0.35 scale factor closely approximates 1/r 
shepherdSpeed = 0.1
destination = np.array([[4],[4]], dtype=np.float32)  # randomly assumed to be (4,4)
thresholdRadius = arenaLength/4
aggregateDone = False

def calcSpeed(distance, costheta):
	speed = 0.5*maxAgentSpeed*np.exp(-expScaleFactor*distance)*(1+costheta)
	return speed

def getUpdatedPosition(position, velocity):
	return position+velocity*timeStep

def returnCentroid(agentPosition):
	return np.array([[np.mean(agentPosition[0,:])], [np.mean(agentPosition[1,:])]])

def main():
	global aggregateDone
	agentPosition = np.random.uniform(low=-arenaLength/2, high=arenaLength/2, size=(2,numberAgents))
	a = list(range(-2, -1))+list(range(1, 2))
	shepherdPosition = np.array([[random.choice(a)+random.uniform(0,1)],[random.choice(a)+random.uniform(0,1)]])
	centroid = returnCentroid(agentPosition)
	shepherdVelocity = np.zeros((2,1), dtype=np.float32)
	agentPositionList = [agentPosition]
	shepherdPositionList = [shepherdPosition]
	centroidList = [centroid]
	shepherdCircleRadius = []

	isClockwise = -1   # randomly assumed any direction for rotation
	count=0
	flag = 1    # flag = 1 corresponds to straight path and flag = 0 to circular path
	rotationMatrix = np.array([[0, isClockwise*1],[-isClockwise*1, 0]], dtype=np.float32)
	while(aggregateDone == False):
		maxDistance = np.max(np.linalg.norm(agentPosition - centroid, axis=0))
		print(maxDistance)
		if(maxDistance <= thresholdRadius):
			aggregateDone = True
			break

		centroidShepherdVector = centroid - shepherdPosition
		centroidShepherdDistance = np.linalg.norm(centroidShepherdVector)
		centroidShepherdVector = centroidShepherdVector/centroidShepherdDistance
		if(centroidShepherdDistance <= maxDistance and flag == 1):
			flag = 0
			print("Circular motion started")
		if(flag == 1):
			shepherdVelocityDirection = centroidShepherdVector
		else:
			shepherdVelocityDirection = np.matmul(rotationMatrix, centroidShepherdVector)
			shepherdCircleRadius.append(centroidShepherdDistance)
		shepherdVelocity = shepherdVelocityDirection*shepherdSpeed

		agentShepherdVector = agentPosition - shepherdPosition
		agentShepherdDistance = np.linalg.norm(agentShepherdVector, axis=0)
		agentShepherdVector = agentShepherdVector/agentShepherdDistance

		costheta = np.matmul(shepherdVelocityDirection.T, agentShepherdVector)
		agentSpeed = np.reshape(calcSpeed(agentShepherdDistance, costheta), (1,numberAgents))
		agentVelocity = agentShepherdVector*agentSpeed

		shepherdPosition = getUpdatedPosition(shepherdPosition, shepherdVelocity)
		agentPosition = getUpdatedPosition(agentPosition, agentVelocity)
		centroid = returnCentroid(agentPosition)
		agentPositionList.append(agentPosition)
		shepherdPositionList.append(shepherdPosition)
		centroidList.append(centroid)
		count+=1

	agentPositionList = np.array(agentPositionList)
	shepherdPositionList = np.array(shepherdPositionList)
	centroidList = np.array(centroidList)
	shepherdCircleRadius = np.array(shepherdCircleRadius)
	
	fig1 = plt.figure(1)
	# plt.plot(destination[0,0], destination[1,0], marker='x')
	plt.plot(agentPositionList[:,0,:], agentPositionList[:,1,:], 'r')
	plt.plot(shepherdPositionList[:,0,0], shepherdPositionList[:,1,0], 'g')
	plt.plot(centroidList[:,0,0], centroidList[:,1,0], 'b')
	circle = plt.Circle((centroid[0,0], centroid[1,0]), thresholdRadius, color='black', fill=False)
	plt.gcf().gca().add_artist(circle)
	plt.grid(True)
	plt.show()

	fig2 = plt.figure(2)
	plt.plot(agentPositionList[:,0,:], agentPositionList[:,1,:], 'r')
	plt.plot(centroidList[:,0,0], centroidList[:,1,0], 'b')
	circle = plt.Circle((centroid[0,0], centroid[1,0]), thresholdRadius, color='black', fill=False)
	plt.gcf().gca().add_artist(circle)
	plt.grid(True)
	plt.show()


	fig3 = plt.figure(3)
	plt.plot(shepherdCircleRadius)
	plt.xlabel('Iterations')
	plt.ylabel('shepherd Circle Motion Radius')
	plt.grid(True)
	plt.show()

if __name__ == '__main__':
	main()