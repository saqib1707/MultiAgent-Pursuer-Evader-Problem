import numpy as np 
import matplotlib.pyplot as plt
import random

i = 0 + random.uniform(0.1, 0.2)
x = []
while i < 10:
	x.append(i)
	i = i+0.1

d_max = 5
ve_max = d_max

x = np.reshape(np.array(x), (len(x),1))

y_threshold = []
for j in range(x.shape[0]):
	if x[j] < d_max:
		y_threshold.append(ve_max - x[j])
	else:
		y_threshold.append(0)

y_threshold = np.array(y_threshold)

y_inverse = 1/x

y_exp = np.exp(-0.5*x)

plt.plot(x, y_threshold, 'green', label='linearly_decreasing (d_max=5)')
plt.plot(x, y_inverse, 'red', label='inverse (k=1)')
plt.plot(x, y_exp, 'blue', label='exponentially_decreasing (k=1)')

plt.title('Comparison of linear, exponentially, inverse relation')
plt.xlabel('pursuer-evader separation (d(e,p,t))')
plt.ylabel('evader speed (v_e)')
plt.legend()
plt.grid(True)
plt.show()