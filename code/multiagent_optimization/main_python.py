import numpy as np
import sys
import os
import nlopt

number_interval = 50
time_interval = 1.0
number_evader = 2
number_pursuer = 1
vemax_repulsion = 0.2
vemax_attraction = 0
vpmax = 0.2
vpmin = 0.05
K = 1.0
epsilon = 0.05

solver = 'fmincon'
algorithm = 'sqp'
max_func_evals = 2e5
max_iter = 1e4
tolfun = 1e-3
tolcon = 1e-3
tolx = 1e-10
num_trial_points = 500
num_stage_one_points = 200


# file = load('../../results_plots/final_attempt_result/hparams/-1_-1_2_2.mat')
# number_interval = file.number_interval

# initial_evader_position = rand(2*number_evader,1)*2-1
# initial_evader_position = file.initial_evader_position
initial_evader_position = np.array([[0.5],[0],[-0.5],[0]])

var = 2*number_pursuer
num_opt_var = var*number_interval

Aineq = []
bineq = []
Aeq = []
beq = []

# initial_pursuer_position = file.initial_pursuer_position
initial_pursuer_position = np.array([[-1],[-1]])

# destination_list = [[11],[10.5],[10],[1-0.5],[1-1],[0.5-1],[0-1],[-0.5-1],[-1-1],[-1-0.5],[-10],[-10.5],[-11],[-0.51],[01],[0.51]]

# destination_list = np.array([[22]]

# destination_list = [[20],[1-2],[0-2],[-1-2],[-2-2],[-2-1],[-20],[-21],[-22],[-12],[02],[12]]

starting_point = np.random.rand(num_opt_var,1)*4-2
destination = np.array([[1],[1]])

opt = nlopt.opt(nlopt.GN_ISRES, num_opt_var)             # Improved Stochastic Ranking Evolution Strategy
opt.set_min_objective(objective_function)

# upper and lower bounds on the parameters
lower_bound = np.full((num_opt_var), -3.0)
upper_bound = np.full((num_opt_var), 3.0)

opt.set_lower_bounds(lower_bound)
opt.set_upper_bounds(upper_bound)

starting_point = np.random.uniform(low=-4.0, high=4.0, size=num_opt_var)

# for i in range(0, N-1):
# 	if(i%var == var-1):
# 		parameter_lower_bound[i] = -input_bound
# 		parameter_upper_bound[i] = input_bound
# 		starting_point[i] = np.random.uniform(low=-1.0, high=1.0)
# upper and lower bounds on the time-duration parameter
# parameter_lower_bound[N-1] = 0.0
# parameter_upper_bound[N-1] = 10.0
# starting_point[N-1] = 5.0    # starting point for delta_t parameter

options = optimoptions(@fmincon, 'Algorithm', algorithm, 'MaxFunEvals', max_func_evals, ...
'MaxIter', max_iter, 'TolFun', tolfun, 'TolCon', tolcon, 'TolX', tolx, 'Display', 'off', ... 
'GradObj', 'off', 'DerivativeCheck', 'on', 'FinDiffType', 'central', 'FunValCheck', 'on', 'Diagnostics', 'off',...
'UseParallel', true)

obj_func = @(x)objective_function(x, number_interval, var, initial_pursuer_position)

nonlinearcons = @(x)non_linear_constraints(x, var, number_interval, number_evader, time_interval, ...
    initial_evader_position, initial_pursuer_position, vemax_repulsion, vemax_attraction, ...
    vpmax, vpmin, epsilon, K, destination)

# [opt_x, fval, exitflag, output] = fmincon(obj_func, starting_point, A, b, ...
# Aeq, beq, lower_bound, upper_bound, nonlinearcons, options)
# [opt_x] = patternsearch(obj_func, starting_point, A, b, Aeq, beq, lower_bound, ...
# upper_bound, nonlinearcons, options)

problem = createOptimProblem(solver, 'objective', obj_func, 'x0', starting_point, 'Aeq', Aeq, 'beq', ...
    beq, 'Aineq', Aineq, 'bineq', bineq, 'lb', lower_bound, 'ub', upper_bound, 'nonlcon', ...
    nonlinearcons, 'options', options)

gs = GlobalSearch('NumTrialPoints', num_trial_points, 'NumStageOnePoints', num_stage_one_points, 'Display', 'iter')
[opt_x, fval, exitflag, outputs] = run(gs, problem)

pursuer_optimized_trajectory = horzcat(initial_pursuer_position, reshape(opt_x, var, number_interval))
evader_optimized_trajectory = compute_evader_position(pursuer_optimized_trajectory, number_evader, initial_evader_position, ...
    number_interval, time_interval, vemax_repulsion, vemax_attraction, K)

# computing pursuer velocity
# pursuer_velocity = zeros(number_interval-1,1)
# for t = 1:number_interval-1
#     pursuer_velocity(t,1) = norm(pursuer_optimized_trajectory(:,t+1) - pursuer_optimized_trajectory(:,t))
# end

# h0 = figure
# plot(pursuer_optimized_trajectory(1,:), pursuer_optimized_trajectory(2,:), '.-', 'color', 'blue', 'LineWidth', 1)hold on
# %     for i = 1:number_evader
# %         plot(evader_optimized_trajectory(2*i-1,1), evader_optimized_trajectory(2*i,1), '.-', 'color', 'green')hold on
# %         plot(evader_optimized_trajectory(2*i-1,2:number_interval), evader_optimized_trajectory(2*i,2:number_interval), '.-', 'color', 'yellow')hold on
# %         plot(evader_optimized_trajectory(2*i-1,number_interval+1), evader_optimized_trajectory(2*i,number_interval+1), '.-', 'color', 'red')hold on
# %     end
# for t = 1:number_interval+1 
#    plot([evader_optimized_trajectory(1,t), evader_optimized_trajectory(3,t)], [evader_optimized_trajectory(2,t), evader_optimized_trajectory(4,t)], ...
#        'color', 'red', 'LineWidth', 1)
# end
# draw_circle(destination(1,1), destination(2,1), epsilon)
# grid on
# xlabel('X')
# ylabel('Y')
# title('shepherding-optimization-result')
# hold off

# %     if or((exitflag == 1), (exitflag == 2))
#     filename = strcat(num2str(initial_pursuer_position(1,1)), '_', num2str(initial_pursuer_position(2,1)), '_', ...
#         num2str(destination(1,1)), '_', num2str(destination(2,1)))

#     savefilename_fig = strcat('../../results_plots/final_attempt_result/fig_format/', filename, '.fig')
#     savefilename_png = strcat('../../results_plots/final_attempt_result/png_format/', filename, '.png')
#     savefig(h0, savefilename_fig)
#     saveas(h0, savefilename_png)

#     savefilename_hparams = strcat('../../results_plots/final_attempt_result/hparams/', filename, '.mat')
#     save(savefilename_hparams, 'hp')
# %     end
# close

# elapsed_time = toc
# disp(strcat('Elapsed Time:', num2str(elapsed_time)))
# end