import numpy as np 
import matplotlib.pyplot as plt
import random

i = 0 + random.uniform(0.5, 0.6)
x = []
while i<30:
	x.append(i)
	i = i+0.2

x = np.reshape(np.array(x), (len(x),1))
y1 = np.exp(-0.50*x)
y2 = 1/x
y3 = 1/(x**2)

fig1 = plt.figure(1)
plt.plot(x, y1, 'r')
plt.plot(x, y2, 'y')
plt.plot(x, y3, 'b')
plt.text(18, 2.5, r'$red : exp,yellow : 1/r,blue : 1/r^2$')
plt.grid(True)
fig1.suptitle('Comparison of exponential,inverse,inverse square with expScaleFactor=0.50')
plt.show()