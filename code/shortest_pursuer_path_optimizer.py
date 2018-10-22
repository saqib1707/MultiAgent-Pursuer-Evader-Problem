import numpy as np 
import matplotlib.pyplot as plt 
import nlopt
import math
import pdb

time_interval = 0.1
evader_max_velocity = 1.0
pursuer_vel_lower = 0.1
k2 = 1.0
number_evaders = Ne = 3
number_intervals = Ni = 5
epsilon = 0.01
tolerance = 1e-8
max_eval = 2000
N = (Ne+1)*2*(Ni+1)    # dimension of the optimization problem

def objective_function(parameter_list, grad):
	path_length = 0
	for i in range(1, Ni+1):
		index1 = (Ne*2 + 2)*i + (Ne*2)
		index2 = (Ne*2 + 2)*(i-1) + (Ne*2)
		path_length += np.sum(np.square((parameter_list[index1:index1+2]-parameter_list[index2:index2+2])/time_interval))
	path_length = path_length*time_interval
	return path_length

def inequality_constraint_1(parameter_list, i):   # i represents the evader numbers
	centroid = np.zeros(2)
	for j in range(Ne):
		index1 = Ni*(Ne*2 + 2) + 2*j
		centroid = centroid + parameter_list[index1:index1+2]
	centroid = centroid/Ne
	index1 = Ni*(Ne*2 + 2) + 2*i
	return np.square(np.linalg.norm(parameter_list[index1:index1+2] - centroid)) - epsilon

def inequality_constraint_2(parameter_list, t):
	index1 = t*(Ne*2 + 2) + (Ne*2)
	index2 = (t-1)*(Ne*2 + 2) + (Ne*2)
	return pursuer_vel_lower - np.linalg.norm((parameter_list[index1:index1+2] - parameter_list[index2:index2+2])/time_interval)

def equality_constraint(parameter_list, i, t, index):    # i represents the evader number
	index0 = (t-1)*(Ne*2 + 2) + 2*i
	index1 = t*(Ne*2 + 2) + 2*i
	index2 = t*(Ne*2 + 2) + (Ne*2)
	index3 = (t-1)*(Ne*2 + 2) + (Ne*2)

	term0 = parameter_list[index1:index1+2] - parameter_list[index2:index2+2]
	term1 = np.exp(-k2*np.linalg.norm(term0))
	term2 = (parameter_list[index2:index2+2] - parameter_list[index3:index3+2])/time_interval
	term4 = 1 + (np.matmul(term2.T, term0)/(np.linalg.norm(term2)*np.linalg.norm(term0)))
	term5 = term0/np.linalg.norm(term0)
	term6 = (parameter_list[index1:index1+2] - parameter_list[index0:index0+2])/time_interval
	return (evader_max_velocity/2)*term1*term4*term5[index] - term6[index]

opt = nlopt.opt(nlopt.LN_COBYLA, N)
opt.set_min_objective(objective_function)

parameter_lower_bound = -np.ones(N)
for i in range(N):
	index1 = Ne*2 + 2
	if(i%index1 == index1-1 or i%index1 == index1-2):
		parameter_lower_bound[i] = -float("inf")

parameter_upper_bound = -parameter_lower_bound
opt.set_lower_bounds(parameter_lower_bound)
opt.set_upper_bounds(parameter_upper_bound)

for i in range(Ne):
	opt.add_inequality_constraint(lambda parameter_list, grad:inequality_constraint_1(parameter_list, i), tolerance)
	for t in range(1, Ni+1):
		for index in range(2):
			opt.add_equality_constraint(lambda parameter_list, grad:equality_constraint(parameter_list, i, t, index), 0)

for t in range(1, Ni+1):
	opt.add_inequality_constraint(lambda parameter_list, grad:inequality_constraint_2(parameter_list, t), tolerance)

opt.set_maxeval(max_eval)
opt.set_xtol_rel(tolerance)

starting_point = np.random.uniform(low=-1.0, high=1.0, size=N)
optimal_parameters = opt.optimize(starting_point)
optimum_path_length = opt.last_optimum_value()

pursuer_position = []
for i in range(Ni+1):
	index = i*(Ne*2+2) + Ne*2
	pursuer_position.append(optimal_parameters[index:index+2])

all_evader_position = []
for i in range(Ne):
	evader_position = []
	for t in range(Ni+1):
		evader_position.append(optimal_parameters[t*(Ne*2 + 2) + 2*i : t*(Ne*2+2) + 2*i + 2])
	all_evader_position.append(np.array(evader_position))

all_evader_position = np.array(all_evader_position)
pursuer_position = np.array(pursuer_position)

plt.figure(1)
for i in range(Ne):
	plt.plot(all_evader_position[i,0,0], all_evader_position[i,0,1], marker='.', color='r')
	plt.plot(all_evader_position[i,Ni,0], all_evader_position[i,Ni,1], marker='.', color='g')
plt.plot(pursuer_position[:,0], pursuer_position[:,1], marker='.', color='b')
plt.grid(True);plt.xlabel("x");plt.ylabel("y")
plt.show()
print("Done")