import numpy as np 
import nlopt 
import matplotlib.pyplot as plt 

# time_interval = 1.0
v = 1.0
u = 
epsilon = 0.1
u_max = 1.0
number_intervals = 100

def objective_function():
	return number_intervals*time_interval

def inequality_constraints():
	return x[number_intervals*time_interval]**2 + y[number_intervals*time_interval]**2 - epsilon

def equality_constraints():
	