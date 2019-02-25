clear; clc;

% hyper-parameters
number_interval = 20;
time_interval = 1.0;
number_evader = 2;
number_pursuer = 1;
vemax_repulsion = 0.5;
vemax_attraction = 0;
vpmax = 0.5;
vpmin = 0.1;
K = 1.0;
epsilon = 0.05;

var = 2*number_pursuer;
N = var*number_interval;

% initial_evader_position = rand(2*number_evader,1)*2-1;
initial_evader_position = [-1;0;1;0];
initial_pursuer_position = [-1;-1];
starting_point = rand(N,1)*2-1;

% file = load('data_file.mat');
% starting_point = file.starting_point;
% initial_point = file.initial_point;

destination = [1;1];
lower_bound(1:N,1) = -100;
upper_bound(1:N,1) = 100;

A = [];
b = [];
Aeq = [];
beq = [];

options = optimoptions('fmincon', 'Algorithm', 'sqp', ...
'MaxFunEvals', 200000, 'MaxIter', 10000, 'TolFun', 1e-2, 'TolCon', 1e-2, 'TolX', 1e-12, ...
'Display', 'iter', 'GradObj', 'off', 'DerivativeCheck','off', 'FinDiffType', 'central');

obj_func = @(x)objective_function(x,number_interval,var,initial_pursuer_position);
nonlinearcons = @(x)constraints(x,var,number_interval,number_evader,time_interval,initial_evader_position,initial_pursuer_position,vemax_repulsion,vemax_attraction,vpmax,vpmin,epsilon,K,destination);
[opt_x, fval, exitflag, output] = fmincon(obj_func,starting_point,A,b,Aeq,beq,lower_bound,upper_bound,nonlinearcons,options);

pursuer_position = horzcat(initial_pursuer_position,reshape(opt_x,var,number_interval));
evader_position = compute_evader_position(pursuer_position,number_evader,initial_evader_position,number_interval,time_interval,vemax_repulsion,vemax_attraction,K);

figure;
plot(pursuer_position(1,:), pursuer_position(2,:), 'o-', 'color', 'blue');hold on;
for i=1:number_evader
    plot(evader_position(2*i-1,1),evader_position(2*i,1), 'o-', 'color', 'green');hold on;
    plot(evader_position(2*i-1,2:number_interval),evader_position(2*i,2:number_interval),'o-','color','yellow');hold on;
    plot(evader_position(2*i-1,number_interval+1),evader_position(2*i,number_interval+1), 'o-', 'color', 'red');hold on;
end
% final_centroid = mean(reshape(evader_position(:,number_interval+1),2,number_evader),2);
draw_circle(destination(1,1), destination(2,1), epsilon);
grid on;
xlabel('X');
ylabel('Y');
title('shepherding-optimization-result');
hold off;