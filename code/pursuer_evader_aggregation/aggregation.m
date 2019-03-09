clear; clc;

% ---------------------hyper-parameters------------------------
hp.number_interval = 25;
hp.time_interval = 1.0;
hp.number_evader = 2;
hp.number_pursuer = 1;
hp.vemax_repulsion = 0.5;
hp.vemax_attraction = 0;
hp.vpmax = 0.5;
hp.vpmin = 0.05;
hp.K = 1.0;
hp.epsilon = 0.05;

hp.solver = 'fmincon';
hp.algorithm = 'sqp';
hp.max_func_evals = 1e5/2;
hp.max_iter = 1e4;
hp.tolfun = 1e-6;
hp.tolcon = 1e-6;
hp.tolx = 1e-10;

hp.var = 2*(hp.number_evader+hp.number_pursuer);
hp.N = hp.var*hp.number_interval;

hp.destination = [1;1];
hp.lower_bound(1:hp.N,1) = -5.0;
hp.upper_bound(1:hp.N,1) = 5.0;

% file = load('data_file.mat');

% hp.initial_point = rand(hp.var,1)*2-1;
% hp.initial_point(hp.var-1:hp.var,1) = [-1;-1];
hp.initial_point = [-0.5;0;0.5;0;-1;-1];
% hp.initial_point = file.initial_point;

hp.starting_point = rand(hp.N,1)*2-1;
% hp.starting_point = file.starting_point;

hp.A = [];
hp.b = [];
hp.Aeq = [];
hp.beq = [];
% ---------------------------hyper-parameters------------------------

options = optimoptions(hp.solver, 'Algorithm', hp.algorithm, ...
'MaxFunEvals', hp.max_func_evals, 'MaxIter', hp.max_iter, 'TolFun', hp.tolfun, 'TolCon', hp.tolcon, 'TolX', hp.tolx, ...
'Display', 'iter', 'GradObj', 'off', 'DerivativeCheck', 'off', 'FinDiffType', 'central');

obj_func = @(x)objective_function(x, hp.number_interval, hp.var, hp.initial_point);
nonlinearcons = @(x)constraints(x, hp.number_interval, hp.time_interval, hp.number_evader, hp.vemax_repulsion, hp.vemax_attraction, hp.vpmax, hp.vpmin, hp.K, hp.epsilon, hp.var, hp.initial_point, hp.destination);
[hp.opt_x, hp.fval, hp.exitflag, hp.output] = fmincon(obj_func, hp.starting_point, hp.A, hp.b, hp.Aeq, hp.beq, hp.lower_bound, hp.upper_bound, nonlinearcons, options);

optimized_parameters = horzcat(hp.initial_point,reshape(hp.opt_x,hp.var,hp.number_interval));
pursuer_position = optimized_parameters(hp.var-1:hp.var,:);
evader_position = optimized_parameters(1:hp.number_evader*2,:);

figure;
plot(pursuer_position(1,:), pursuer_position(2,:), 'o-', 'color', 'blue');hold on;
for i=1:hp.number_evader
    plot(evader_position(2*i-1,1),evader_position(2*i,1), 'o-', 'color', 'green');hold on;
    plot(evader_position(2*i-1,2:hp.number_interval),evader_position(2*i,2:hp.number_interval),'o-','color','yellow');hold on;
    plot(evader_position(2*i-1,hp.number_interval+1),evader_position(2*i,hp.number_interval+1), 'o-', 'color', 'red');hold on;
end
draw_circle(hp.destination(1,1), hp.destination(2,1), hp.epsilon);
grid on;
xlabel('X');
ylabel('Y');
title('shepherding-optimization-result');
hold off;

hp.norm_error = evader_path_verify(hp.number_evader,hp.initial_point,hp.number_interval,hp.time_interval,hp.opt_x,hp.vemax_repulsion,hp.vemax_attraction,hp.K,hp.var,hp.epsilon,hp.destination);