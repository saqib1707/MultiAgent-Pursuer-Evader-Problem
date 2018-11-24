import numpy as np
import matplotlib.pyplot as plt
import nlopt
import pdb

number_interval = 20
epsilon = 1e-4
max_eval = 1500000
tolerance = 1e-8
var = 2
N = var*number_interval
initial_point = np.array([5.0, 5.0], dtype=np.float32)
count_eval = 0

def objective_function(x, grad):
	global count_eval
	count_eval += 1
	path_length = np.linalg.norm(x[0:2] - initial_point)
	for i in range(1, number_interval):
		path_length += np.linalg.norm(x[var*i:var*i+2] - x[var*(i-1):var*(i-1)+2])
	# print(count_eval, path_length)
	return path_length

def inequality_constraint_1(x):
	return (np.square(x[var*(number_interval-1)]) + np.square(x[var*(number_interval-1)+1]) - epsilon)

opt = nlopt.opt(nlopt.GN_ISRES, N)
opt.set_min_objective(objective_function)

# upper and lower bounds on the parameters
parameter_lower_bound = -5*np.ones(N)
parameter_upper_bound = 5*np.ones(N)

opt.set_lower_bounds(parameter_lower_bound)
opt.set_upper_bounds(parameter_upper_bound)

opt.add_inequality_constraint(lambda x, grad:inequality_constraint_1(x), tolerance)

opt.set_maxeval(max_eval)
opt.set_xtol_rel(tolerance)

starting_point = np.random.uniform(low=-5.0, high=5.0, size=N)
opt_parameter = opt.optimize(starting_point)
print("Final Optimized distance to reach near origin : ", opt.last_optimum_value())

x_coord = [initial_point[0]]
y_coord = [initial_point[1]]

for i in range(0, number_interval):
	x_coord.append(opt_parameter[var*i])
	y_coord.append(opt_parameter[var*i+1])

x_coord = np.array(x_coord)
y_coord = np.array(y_coord)

plt.figure(1)
plt.plot(x_coord, y_coord, marker='.');plt.grid(True);plt.xlabel("x");plt.ylabel("y");plt.xlim((-5,5));plt.ylim((-5,5))
circle = plt.Circle((0, 0), np.sqrt(epsilon), color='r', fill=False)
ax = plt.gca()
ax.add_artist(circle)
plt.show()
print("Done")