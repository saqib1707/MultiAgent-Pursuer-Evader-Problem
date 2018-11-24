import numpy as np
import matplotlib.pyplot as plt
import nlopt
import pdb

number_interval = 10
v = 1.0
epsilon = 1e-4
max_eval = 1500000
tolerance = 1e-8
N = 3*number_interval
var = 3
count_eval = 0
initial_point = np.array([1.0, 1.0, 90.0], dtype=np.float32)
angle_deviation = 1       # in degrees

def objective_function(x, grad):
	global count_eval
	count_eval += 1
	path_length = np.linalg.norm(x[0:var-1] - initial_point[0:var-1])
	for i in range(1, number_interval):
		path_length += np.linalg.norm(x[var*i:var*i+2] - x[var*(i-1):var*(i-1)+2])
	print(count_eval, path_length)
	return path_length

def inequality_constraint_1(x):
	return (np.square(x[var*(number_interval-1)]) + np.square(x[var*(number_interval-1)+1]) - epsilon)

def inequality_constraint_2(x, time_step):
	if time_step == 1:
		return (x[var*time_step-1] - initial_point[2] - angle_deviation)
	else:
		return (x[var*time_step-1] - x[var*(time_step-1)-1] - angle_deviation)

def inequality_constraint_3(x, time_step):
	if time_step == 1:
		return (-angle_deviation - (x[var*time_step-1] - initial_point[2]))
	else:
		return (-angle_deviation - (x[var*time_step-1] - x[var*(time_step-1)-1]))

def equality_constraint_1(x, time_step):
	if time_step == 1:
		return x[var*(time_step-1)] - initial_point[0] - v*np.cos(initial_point[2]*np.pi/180.0)
	else:
		return x[var*(time_step-1)] - x[var*(time_step-2)] - v*np.cos(x[var*(time_step-1)-1]*np.pi/180.0)

def equality_constraint_2(x, time_step):
	if time_step == 1:
		return x[var*(time_step-1)+1] - initial_point[1] - v*np.sin(initial_point[2]*np.pi/180.0)
	else:
		return x[var*(time_step-1)+1] - x[var*(time_step-2)+1] - v*np.sin(x[var*(time_step-1)-1]*np.pi/180.0)

opt = nlopt.opt(nlopt.GN_ISRES, N)
opt.set_min_objective(objective_function)

# upper and lower bounds on the parameters
parameter_lower_bound = np.zeros(N)
parameter_upper_bound = np.ones(N)
for i in range(0, N):
	if(i%var == var-1):
		parameter_lower_bound[i] = 0
		parameter_upper_bound[i] = 360.0

# upper and lower bounds on the time-duration parameter
# parameter_lower_bound[var*number_interval] = 1e-8
# parameter_upper_bound[var*number_interval] = 10

opt.set_lower_bounds(parameter_lower_bound)
opt.set_upper_bounds(parameter_upper_bound)

opt.add_inequality_constraint(lambda x, grad:inequality_constraint_1(x), tolerance)
for time_step in range(1, number_interval+1):
	opt.add_inequality_constraint(lambda x, grad:inequality_constraint_2(x, time_step), tolerance)
	opt.add_inequality_constraint(lambda x, grad:inequality_constraint_3(x, time_step), tolerance)
	opt.add_equality_constraint(lambda x, grad:equality_constraint_1(x, time_step), 0)
	opt.add_equality_constraint(lambda x, grad:equality_constraint_2(x, time_step), 0)

opt.set_maxeval(max_eval)
opt.set_xtol_rel(tolerance)

starting_point = np.random.uniform(low=0, high=1.0, size=N)
opt_parameter = opt.optimize(starting_point)
print("Final Optimized time to reach near origin : ", opt.last_optimum_value())

x_coord = [initial_point[0]]
y_coord = [initial_point[1]]

for i in range(0, number_interval):
	x_coord.append(opt_parameter[var*i])
	y_coord.append(opt_parameter[var*i+1])

x_coord = np.array(x_coord)
y_coord = np.array(y_coord)

plt.figure(1)
plt.plot(x_coord, y_coord, marker='.');plt.grid(True);plt.xlabel("x");plt.ylabel("y");plt.xlim((-1,1));plt.ylim((-1.5,1.5))
circle = plt.Circle((0, 0), np.sqrt(epsilon), color='r', fill=False)
ax = plt.gca()
ax.add_artist(circle)
plt.show()
print(count_eval, "Done")