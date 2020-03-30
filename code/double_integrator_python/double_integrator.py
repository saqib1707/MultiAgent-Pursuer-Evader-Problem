import numpy as np
import matplotlib.pyplot as plt
import nlopt
import pdb

number_interval = 30
epsilon = 1e-2
input_bound = 1.0
max_eval = 200000
tolerance = 1e-6
var = 3
N = var*number_interval+1
count_eval = 0
initial_point = np.array([4.0, 4.0, 0.5], dtype=np.float32)

def objective_function(x, grad):
	global count_eval
	count_eval += 1
	total_time = number_interval*x[N-1]
	# print(count_eval, total_time)
	return total_time

def inequality_constraint_1(x):
	return (np.square(x[var*(number_interval-1)]) + np.square(x[var*(number_interval-1)+1]) - epsilon)

def equality_constraint_1(x, t):
	if t == 1:
		return x[var*(t-1)] - initial_point[0] - initial_point[1]*x[N-1]
	else:
		return x[var*(t-1)] - x[var*(t-2)] - x[var*(t-2)+1]*x[N-1]

def equality_constraint_2(x, t):
	if t == 1:
		return x[var*(t-1)+1] - initial_point[1] - initial_point[2]*x[N-1]
	else:
		return x[var*(t-1)+1] - x[var*(t-2)+1] - x[var*(t-2)+2]*x[N-1]

opt = nlopt.opt(nlopt.GN_ISRES, N)             # Improved Stochastic Ranking Evolution Strategy
opt.set_min_objective(objective_function)

# upper and lower bounds on the parameters
parameter_lower_bound = -np.full((N), 2e6)
parameter_upper_bound = np.full((N), 2e6)
starting_point = np.random.uniform(low=-4.0, high=4.0, size=N)
for i in range(0, N-1):
	if(i%var == var-1):
		parameter_lower_bound[i] = -input_bound
		parameter_upper_bound[i] = input_bound
		starting_point[i] = np.random.uniform(low=-1.0, high=1.0)
# upper and lower bounds on the time-duration parameter
parameter_lower_bound[N-1] = 0.0
parameter_upper_bound[N-1] = 10.0
starting_point[N-1] = 5.0    # starting point for delta_t parameter

opt.set_lower_bounds(parameter_lower_bound)
opt.set_upper_bounds(parameter_upper_bound)

opt.add_inequality_constraint(lambda x, grad:inequality_constraint_1(x), tolerance)
for time_step in range(1, number_interval+1):
	opt.add_equality_constraint(lambda x, grad:equality_constraint_1(x, time_step), 0)
	opt.add_equality_constraint(lambda x, grad:equality_constraint_2(x, time_step), 0)

opt.set_maxeval(max_eval)
opt.set_xtol_rel(tolerance)

opt_parameter = opt.optimize(starting_point)
print("Optimized Time Interval : ", opt_parameter[N-1])

x_coord = [initial_point[0]]
y_coord = [initial_point[1]]
theta_coord = [initial_point[2]]

for i in range(0, number_interval):
	x_coord.append(opt_parameter[var*i])
	y_coord.append(opt_parameter[var*i+1])
	theta_coord.append(opt_parameter[var*i+2])

x_coord = np.array(x_coord)
y_coord = np.array(y_coord)
theta_coord = np.array(theta_coord)

plt.figure(1)
plt.plot(x_coord, y_coord, marker='.');plt.grid(True);plt.xlabel("x");plt.ylabel("y");plt.xlim((-5,5));plt.ylim((-5,5))
circle = plt.Circle((0, 0), np.sqrt(epsilon), color='r', fill=False)
ax = plt.gca()
ax.add_artist(circle)
plt.show()
# plt.figure(2)
# plt.plot(theta_coord,marker='.');plt.grid(True);plt.xlabel("intervals");plt.ylabel("theta");
# plt.show()
print("Done")