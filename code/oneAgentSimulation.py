import numpy as np
import matplotlib.pyplot as plt
import pdb

numberAgents = 1
timeStep = 1.0
arenaLength = 2
shepherdSpeed = 10.0
maxAgentSpeed = 5.0
distance_threshold = 10.0*arenaLength*np.sqrt(2)
# m = 1.5
factor = 5
# alpha = 2
accuracy = 0.9995
epsilon = 0
flag = False

def calcSpeed(distance):
	# speed = (maxAgentSpeed-m*distance) if maxAgentSpeed>=m*distance else 0
	# speed = shepherdSpeed/(10*pow(distance,alpha)) if distance > 1.0 else shepherdSpeed/10 
	speed = maxAgentSpeed*np.exp(-distance) #if distance < distance_threshold else epsilon 
	return speed

def getUpdatedPosition(position, velocity):
	return position+velocity*timeStep

def main():
	count = 0
	global flag
	destination = np.zeros((2,1))
	agentPosition = np.random.uniform(low=-arenaLength/2, high=arenaLength/2, size=(2,1))
	shepherdPosition = np.random.uniform(low=-arenaLength/2, high=arenaLength/2, size=(2,1))

	shepherdVelocity = np.zeros((2,1), dtype=np.float32)
	agentPositionList = [agentPosition]
	shepherdPositionList = [shepherdPosition]
	Xda = agentPosition-destination
	Xds = shepherdPosition-destination
	Xda = Xda/np.linalg.norm(Xda)
	Xds = Xds/np.linalg.norm(Xds)
	if(np.matmul(Xda.T,Xds)>accuracy):
		print("Initially Already in line")
	else:
		while(True):
			lineOfSightVector = shepherdPosition-agentPosition
			# radius = np.sqrt(np.matmul(lineOfSightVector.T, lineOfSightVector))
			dsa = np.linalg.norm(lineOfSightVector)
			lineOfSightVector = lineOfSightVector/dsa
			agentSpeed = calcSpeed(dsa)
			agentVelocity = -agentSpeed*lineOfSightVector
			# corresponds to motion when shepherd moves in straight line opposite
			# to the agent
			if(agentSpeed > shepherdSpeed/10.0):
				count+=1
				shepherdVelocity = shepherdSpeed*lineOfSightVector
			else:
				# shepherd does clockwise rotation
				if(shepherdPosition[0,0] < agentPosition[0,0]):
					sign = 1
				else:
					sign = -1
				slope = (shepherdPosition[1,0]-agentPosition[1,0])/(shepherdPosition[0,0]-agentPosition[0,0])
				shepherdVelocity[1,0] = sign*shepherdSpeed/np.sqrt(1+pow(slope,2))
				shepherdVelocity[0,0] = -slope*shepherdVelocity[1,0]
				Xda = agentPosition-destination
				Xds = shepherdPosition-destination
				Xda = Xda/np.linalg.norm(Xda)
				Xds = Xds/np.linalg.norm(Xds)
				if(np.matmul(Xda.T,Xds)>accuracy):
					flag = True
			shepherdPosition = getUpdatedPosition(shepherdPosition, shepherdVelocity)
			agentPosition = getUpdatedPosition(agentPosition, agentVelocity)
			agentPositionList.append(agentPosition)
			shepherdPositionList.append(shepherdPosition)
			if(flag):
				break

	# print(count)
	agentPositionList = np.array(agentPositionList)
	shepherdPositionList = np.array(shepherdPositionList)

	plt.figure(1)
	# print(shepherdPositionList)
	plt.plot(destination[0,0],destination[1,0], marker='x')
	plt.plot(agentPositionList[:,0,0], agentPositionList[:,1,0], 'r', marker='.')
	plt.plot(shepherdPositionList[:,0,0], shepherdPositionList[:,1,0], 'b')
	plt.plot([shepherdPositionList[-1,0,0],destination[0,0]],[shepherdPositionList[-1,1,0],destination[1,0]], 'g--')
	# plt.arrow(shepherdPositionList[5,0,0], shepherdPositionList[5,1,0], shepherdPositionList[6,0,0], shepherdPositionList[6,1,0], shape='full', lw=0, length_includes_head=True, head_width=.05)
	
	# plt.ylim((-arenaLength*factor, +arenaLength*factor))
	# plt.xlim((-arenaLength*factor, +arenaLength*factor))
	plt.grid(True)
	plt.show()

if __name__=='__main__':
	main()