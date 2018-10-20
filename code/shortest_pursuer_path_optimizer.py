import numpy as np 
import matplotlib.pyplot as plt 
import nlopt
import pdb

time_interval = 1.0
number_intervals = 100
evader_max_velocity = 1.0
k2 = 1.0
number_evaders = 10
pursuer_position = np.random.uniform(low=1.0, high=5.0, size=(number_intervals+1, 2))
evader_position = np.random.uniform(low=-1.0, high=1.0, size=(number_intervals+1, number_evaders, 2))
centroid = np.zeros((number_intervals+1, 2), dtype=np.float32)
epsilon = 0.1
N = (number_intervals+1)*2            # dimension of the optimization problem

def objective_function(time_step, grad):
	if len(grad) > 0:
		print('Yo')
	path_length = 0
	for i in range(number_intervals+1):
		path_length += np.sum(np.square(pursuer_position[i,:]-pursuer_position[i-1,:]))
	path_length = path_length*time_interval
	return path_length

def inequality_constraint(i, grad):   # i represents the evader number
	if len(grad) > 0:
		print('Yo')
	return np.square(np.linalg.norm(evader_position[number_intervals,i,:] - centroid[number_intervals,:])) - epsilon

def equality_constraint(i, time_step, grad):    # i represents the evader number
	term1 = np.exp(-k2*np.linalg.norm(evader_position[time_step,i,:] - pursuer_position[time_step,:]))
	term2 = pursuer_position[time_step,:] - pursuer_position[time_step-1,:]
	term3 = evader_position[time_step,i,:] - pursuer_position[time_step,:]
	# pdb.set_trace()
	term4 = 1 + (np.matmul(term2.T, term3)/(np.linalg.norm(term2)*np.linalg.norm(term3)))
	term5 = term3/np.linalg.norm(term3)
	term6 = evader_position[time_step,i,:] - evader_position[time_step-1,i,:]
	return (evader_max_velocity/2)*term1*term4*term5 - term6

def main():
	global pursuer_position
	opt = nlopt.opt(nlopt.LN_COBYLA, N)
	print(opt.get_algorithm_name())
	# opt.set_lower_bounds([])
	opt.set_min_objective(objective_function)
	for i in range(number_evaders):
		opt.add_inequality_constraint(lambda pursuer_position, grad:inequality_constraint(i, grad), 1e-8)   # 1e-8 is tolerance value

	# for i in range(number_evaders):
	# 	for t in range(1,number_intervals):
	# 		opt.add_equality_constraint(lambda pursuer_position, grad:equality_constraint(i, t, grad), 0)

	starting_point = np.random.uniform(low=1.0, high=5.0, size=(number_intervals+1, 2))
	optimum_values = opt.optimize(starting_point)
	optimum_path_length = opt.last_optimum_value()

if __name__ == '__main__':
	main()