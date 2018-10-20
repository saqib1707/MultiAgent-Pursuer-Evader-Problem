import numpy as np
import matplotlib.pyplot as plt
import nlopt
import pdb

number_interval = 10
v = 1.0
epsilon = 0.001
N = 3*(number_interval+1)

def calc_time_interval(parameter_list, time_step):
	return (np.linalg.norm(parameter_list[3*time_step:3*time_step+2] - parameter_list[3*(time_step-1):3*(time_step-1)+2]))/v

def objective_function(parameter_list, grad):
	total_time = 0
	for i in range(1, number_interval+1):
		total_time += (np.linalg.norm(parameter_list[3*i:3*i+2] - parameter_list[3*(i-1):3*(i-1)+2]))/v
	return total_time

def inequality_constraint_1(parameter_list):
	return np.square(parameter_list[3*number_interval]) + np.square(parameter_list[3*number_interval+1]) - epsilon

def inequality_constraint_2(parameter_list, time_step):
	time_interval = calc_time_interval(parameter_list, time_step)
	return (parameter_list[3*time_step+2] - parameter_list[3*(time_step-1)+2])/time_interval - 1

def equality_constraint_1(parameter_list, time_step):
	time_interval = calc_time_interval(parameter_list, time_step)
	return (parameter_list[3*time_step] - parameter_list[3*(time_step-1)])/time_interval - v*np.cos(parameter_list[3*time_step+2])

def equality_constraint_2(parameter_list, time_step):
	time_interval = calc_time_interval(parameter_list, time_step)
	return (parameter_list[3*time_step+1] - parameter_list[3*(time_step-1)+1])/time_interval - v*np.sin(parameter_list[3*time_step+2])

opt = nlopt.opt(nlopt.LN_COBYLA, N)
opt.set_min_objective(objective_function)
opt.add_inequality_constraint(lambda parameter_list, grad:inequality_constraint_1(parameter_list), 1e-8)
for time_step in range(1, number_interval+1):
	opt.add_inequality_constraint(lambda parameter_list, grad:inequality_constraint_2(parameter_list, time_step), 1e-8)
	opt.add_equality_constraint(lambda parameter_list, grad:equality_constraint_1(parameter_list, time_step), 0)
	opt.add_equality_constraint(lambda parameter_list, grad:equality_constraint_2(parameter_list, time_step), 0)

opt.set_xtol_rel(1e-8)
starting_point = np.random.uniform(low=0, high=5.0, size=3*(number_interval+1))
opt_parameter = opt.optimize(starting_point)
print("Final Optimized Time to reach near origin : ", opt.last_optimum_value())
# print("Final Optimized Time to reach near origin (manually) : ", objective_function(opt_parameter, []))

x_coord = []
y_coord = []
for i in range(number_interval+1):
	x_coord.append(opt_parameter[3*i])
	y_coord.append(opt_parameter[3*i+1])

x_coord = np.array(x_coord)
y_coord = np.array(y_coord)

plt.figure(1)
plt.plot(x_coord, y_coord);plt.grid(True);plt.xlabel("x");plt.ylabel("y")
plt.show()
print("Done")