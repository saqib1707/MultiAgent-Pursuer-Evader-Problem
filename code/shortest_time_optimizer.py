import numpy as np
import matplotlib.pyplot as plt
import nlopt
import pdb

number_interval = 20
time_interval = 0.1
v = 1.0
epsilon = 0.001
max_eval = 200000
tolerance = 1e-8
N = 3*number_interval+1
count_eval = 0
initial_point = np.array([1.0, 1.0, np.pi/2], dtype=np.float32)

# def objective_function(parameter_list, grad):
# 	path_length = 0
# 	for i in range(1, number_interval+1):
# 		path_length += np.linalg.norm(parameter_list[3*i:3*i+2] - parameter_list[3*(i-1):3*(i-1)+2])
# 	return path_length

def objective_function(parameter_list, grad):
	global count_eval
	count_eval += 1
	total_time = number_interval*parameter_list[3*number_interval]
	return total_time

def inequality_constraint_1(parameter_list):
	return np.square(parameter_list[3*(number_interval-1)]) + np.square(parameter_list[3*(number_interval-1)+1]) - epsilon

def inequality_constraint_2(parameter_list, time_step):
	if time_step == 1:
		return abs((parameter_list[3*time_step-1] - initial_point[2])/parameter_list[3*number_interval]) - 1
	else:
		return abs((parameter_list[3*time_step-1] - parameter_list[3*(time_step-1)-1])/parameter_list[3*number_interval]) - 1

def equality_constraint_1(parameter_list, time_step):
	if time_step == 1:
		return (parameter_list[3*(time_step-1)] - initial_point[0])/parameter_list[3*number_interval] - v*np.cos(parameter_list[3*time_step-1])
	else:
		return (parameter_list[3*(time_step-1)] - parameter_list[3*(time_step-2)])/parameter_list[3*number_interval] - v*np.cos(parameter_list[3*time_step-1])

def equality_constraint_2(parameter_list, time_step):
	if time_step == 1:
		return (parameter_list[3*(time_step-1)+1] - initial_point[1])/parameter_list[3*number_interval] - v*np.sin(parameter_list[3*time_step-1])
	else:
		return (parameter_list[3*(time_step-1)+1] - parameter_list[3*(time_step-2)+1])/parameter_list[3*number_interval] - v*np.sin(parameter_list[3*time_step-1])

opt = nlopt.opt(nlopt.LN_COBYLA, N)
opt.set_min_objective(objective_function)

# upper and lower bounds on the parameters
parameter_lower_bound = -np.ones(N)
parameter_upper_bound = np.ones(N)
for i in range(N):
	index1 = 3
	if(i%index1 == index1-1):
		parameter_lower_bound[i] = 0
		parameter_upper_bound[i] = 2*np.pi

# upper and lower bounds on the time-duration parameter
parameter_lower_bound[3*number_interval] = 1e-8
parameter_upper_bound[3*number_interval] = 100

opt.set_lower_bounds(parameter_lower_bound)
opt.set_upper_bounds(parameter_upper_bound)

opt.add_inequality_constraint(lambda parameter_list, grad:inequality_constraint_1(parameter_list), tolerance)
for time_step in range(1, number_interval+1):
	opt.add_inequality_constraint(lambda parameter_list, grad:inequality_constraint_2(parameter_list, time_step), tolerance)
	opt.add_equality_constraint(lambda parameter_list, grad:equality_constraint_1(parameter_list, time_step), 0)
	opt.add_equality_constraint(lambda parameter_list, grad:equality_constraint_2(parameter_list, time_step), 0)

opt.set_maxeval(max_eval)
opt.set_xtol_rel(tolerance)

starting_point = np.random.uniform(low=0, high=1.0, size=N)
opt_parameter = opt.optimize(starting_point)
print("Final Optimized time to reach near origin : ", opt.last_optimum_value())

x_coord = [initial_point[0]]
y_coord = [initial_point[1]]
for i in range(number_interval):
	x_coord.append(opt_parameter[3*i])
	y_coord.append(opt_parameter[3*i+1])

x_coord = np.array(x_coord)
y_coord = np.array(y_coord)

plt.figure(1)
plt.plot(x_coord, y_coord, marker='.');plt.grid(True);plt.xlabel("x");plt.ylabel("y")
circle = plt.Circle((0, 0), np.sqrt(epsilon), color='r', fill=False)
ax = plt.gca()
ax.add_artist(circle)
plt.show()
print(count_eval, "Done")