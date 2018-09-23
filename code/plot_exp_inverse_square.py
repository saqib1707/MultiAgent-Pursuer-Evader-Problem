import numpy as np 
import matplotlib.pyplot as plt
import random

i = 0 + random.uniform(0.5, 0.6)
x = []
while i<30:
	x.append(i)
	i = i+0.2

x = np.reshape(np.array(x), (len(x),1))
y1 = np.exp(-0.35*x)
y2 = 1/x

fig1 = plt.figure(1)
plt.plot(x, y1, 'r')
plt.plot(x, y2, 'b')
plt.grid(True)
fig1.suptitle('Comparison of exponential vs inverse functions with expScaleFactor=0.35')
plt.show()