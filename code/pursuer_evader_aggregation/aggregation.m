clear; clc;

% ---------------------hyper-parameters------------------------
hp.number_interval = 30;
hp.time_interval = 1.0;
hp.number_evader = 2;
hp.number_pursuer = 1;
hp.vemax_repulsion = 0.4;
hp.vemax_attraction = 0;
hp.vpmax = 0.4;
hp.vpmin = 0.05;
hp.K = 1.0;
hp.epsilon = 0.05;

hp.solver = 'fmincon';
hp.algorithm = 'sqp';
hp.max_func_evals = 2e5;
hp.max_iter = 1e4;
hp.tolfun = 1e-4;
hp.tolcon = 1e-4;
hp.tolx = 1e-12;

hp.var = 2*hp.number_pursuer;
hp.N = hp.var*hp.number_interval;

% file = load('data_file.mat');

% hp.initial_pursuer_position = file.initial_pursuer_position;
hp.initial_pursuer_position = [-1;-1];

% hp.initial_evader_position = rand(2*hp.number_evader,1)*2-1;
% hp.initial_evader_position = file.initial_evader_position;
hp.initial_evader_position = [0.5;0;-0.5;0];

hp.starting_point = rand(hp.N,1)*2-1;
% hp.starting_point = file.starting_point;

hp.destination = [1;1];
hp.lower_bound(1:hp.N,1) = -5.0;
hp.upper_bound(1:hp.N,1) = 5.0;

hp.Aineq = [];
hp.bineq = [];
hp.Aeq = [];
hp.beq = [];
% ---------------------------hyper-parameters------------------------

options = optimoptions(@fmincon, 'Algorithm', hp.algorithm, 'MaxFunEvals', hp.max_func_evals, ...
'MaxIter', hp.max_iter, 'TolFun', hp.tolfun, 'TolCon', hp.tolcon, 'TolX', hp.tolx, 'Display', 'off', ... 
'GradObj', 'off', 'DerivativeCheck','off', 'FinDiffType', 'central');

% options = psoptimset('MaxFunEvals', hp.max_func_evals, 'MaxIter', hp.max_iter, 'TolFun', hp.tolfun, ...
% 'TolCon', hp.tolcon, 'TolX', hp.tolx, 'Display', 'iter', 'PlotFcns', @psplotbestf);

obj_func = @(x)objective_function(x, hp.number_interval, hp.var, hp.initial_pursuer_position);

nonlinearcons = @(x)constraints(x, hp.var, hp.number_interval, hp.number_evader, hp.time_interval, ...
hp.initial_evader_position, hp.initial_pursuer_position, hp.vemax_repulsion, hp.vemax_attraction, ...
hp.vpmax, hp.vpmin, hp.epsilon, hp.K, hp.destination);

% [hp.opt_x, hp.fval, hp.exitflag, hp.output] = fmincon(obj_func, hp.starting_point, hp.A, hp.b, ...
% hp.Aeq, hp.beq, hp.lower_bound, hp.upper_bound, nonlinearcons, options);
% [hp.opt_x] = patternsearch(obj_func, hp.starting_point, hp.A, hp.b, hp.Aeq, hp.beq, hp.lower_bound, ...
% hp.upper_bound, nonlinearcons, options);

problem = createOptimProblem(hp.solver,'objective',obj_func,'x0',hp.starting_point,'Aeq',hp.Aeq,'beq', ...
hp.beq,'Aineq',hp.Aineq,'bineq',hp.bineq,'lb',hp.lower_bound,'ub',hp.upper_bound,'nonlcon', ...
nonlinearcons,'options',options);

gs = GlobalSearch('NumTrialPoints',400,'NumStageOnePoints',200,'Display','iter');
[hp.opt_x,hp.fval,hp.exitflag,hp.outputs] = run(gs,problem);

pursuer_position = horzcat(hp.initial_pursuer_position,reshape(hp.opt_x,hp.var,hp.number_interval));
evader_position = compute_evader_position(pursuer_position,hp.number_evader,hp.initial_evader_position,...
hp.number_interval,hp.time_interval,hp.vemax_repulsion,hp.vemax_attraction,hp.K);

velocity = zeros(hp.number_interval-1,1);
for t = 1:hp.number_interval-1
    velocity(t,1) = norm(pursuer_position(:,t+1) - pursuer_position(:,t));
end

figure;
plot(pursuer_position(1,:), pursuer_position(2,:), 'o-', 'color', 'blue');hold on;
for i=1:hp.number_evader
    plot(evader_position(2*i-1,1),evader_position(2*i,1), 'o-', 'color', 'green');hold on;
    plot(evader_position(2*i-1,2:hp.number_interval),evader_position(2*i,2:hp.number_interval),'o-','color','yellow');hold on;
    plot(evader_position(2*i-1,hp.number_interval+1),evader_position(2*i,hp.number_interval+1), 'o-', 'color', 'red');hold on;
end
for t = 1:hp.number_interval
   plot([evader_position(1,t),evader_position(3,t)],[evader_position(2,t),evader_position(4,t)],'r');
end

draw_circle(hp.destination(1,1), hp.destination(2,1), hp.epsilon);
grid on;
xlabel('X');
ylabel('Y');
title('shepherding-optimization-result');
hold off;