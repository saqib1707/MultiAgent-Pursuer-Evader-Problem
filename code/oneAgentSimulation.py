import numpy as np 
import matplotlib.pyplot as plt 
import pdb


numberAgents = 1
timeStep = 1.0
# alpha = 2
# k = 0.005
shepherdSpeed = 0.1
maxAgentSpeed = 10
numberIteration = 100

flag = False
# sign = -1.0
# prevSlope = 5

def calcSpeed(distance):
	speed = (maxAgentSpeed-distance) if maxAgentSpeed>=distance else 0
	return speed

def getUpdatedPosition(position, velocity):
	return position+velocity*timeStep

def main():
	global flag
	destination = np.random.uniform(low=0.0, high=10.0, size=(2,1))
	agentPosition = np.random.uniform(low=0.0, high=10.0, size=(2,1))
	shepherdPosition = np.random.uniform(low=0.0, high=10.0, size=(2,1))

	# radius = np.sqrt(np.matmul(np.transpose(destination-shepherdPosition), (destination-shepherdPosition)))
	shepherdVelocity = np.zeros((2,1), dtype=np.float32)
	agentPositionList = [agentPosition]
	shepherdPositionList = [shepherdPosition]
	count = 0
	while(count < numberIteration):
		radius = np.sqrt(np.matmul(np.transpose(agentPosition-shepherdPosition), (agentPosition-shepherdPosition)))
		agentSpeed = calcSpeed(radius)
		lineOfSightVector = agentPosition-shepherdPosition
		# unitVector = np.linalg.norm(lineOfSightVector)
		lineOfSightVector = lineOfSightVector/np.linalg.norm(lineOfSightVector)
		agentVelocity = agentSpeed*lineOfSightVector
		print(agentVelocity)


		# print(radius)
		try:
			slope = (shepherdPosition[1,0]-agentPosition[1,0])/(shepherdPosition[0,0]-agentPosition[0,0])
			# print(slope)
		except RuntimeWarning:
			flag = True
		
		# if (slope < 0 and prevSlope > 0):
		# 	sign = -1*sign

		if(flag == False):
			C = pow(shepherdSpeed, 2)-2*slope*agentVelocity[1,0]*agentVelocity[0,0]-pow((slope*agentVelocity[1,0]),2)-pow(agentVelocity[0,0],2)
			print(pow((pow(slope,2)*agentVelocity[1,0]+slope*agentVelocity[0,0]),2)+C*(pow(slope,2)+1))
			shepherdVelocity[1,0] = (slope*(slope*agentVelocity[1,0]+agentVelocity[0,0])+np.sqrt(pow((pow(slope,2)*agentVelocity[1,0]+slope*agentVelocity[0,0]),2)+C*(pow(slope,2)+1)))/(pow(slope,2)+1)
			shepherdVelocity[0,0] =  agentVelocity[0,0]-slope*(shepherdVelocity[1,0]-agentVelocity[1,0])
		# else:
		# 	if(shepherdPosition[1,0] < [1,0]):
		# 		shepherdVelocity = np.array([[shepherdSpeed],[0]], dtype=np.float32)
		# 	else:
		# 		shepherdVelocity = np.array([[-shepherdSpeed],[0]], dtype=np.float32)
		# 	flag = False
		
		# distance = np.reshape(np.linalg.norm(lineOfSightVector, axis=1), (2,1))
		# force = getForce(distance)
		# agentSpeed = force
		# agentVelocity = agentSpeed*(lineOfSightVector/distance)
		# pdb.set_trace()
		#update step
		shepherdPosition = getUpdatedPosition(shepherdPosition, shepherdVelocity)
		agentPosition = getUpdatedPosition(agentPosition, agentVelocity)
		agentPositionList.append(agentPosition)
		shepherdPositionList.append(shepherdPosition)
		# prevSlope = slope
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

	# pdb.set_trace()
	# print(agentPositionList)
	plt.figure(1)
	plt.plot(agentPositionList[:,0,0], agentPositionList[:,1,0], 'r--')
	# plt.plot(agentPositionList[:,1,0], agentPositionList[:,1,1], 'g--')
	# plt.plot(shepherdPositionList[:,0,0], shepherdPositionList[:,1,0], 'b--')
	plt.grid(True)
	plt.show()

if __name__=='__main__':
	main()