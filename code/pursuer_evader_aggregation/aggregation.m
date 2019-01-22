clear; clc;

% tuning hyper-parameters
number_interval = 30;
time_interval = 1.0;
number_evader = 2;
vemax_repulsion = 1.0;
vemax_attraction = 1.0;
vpmax = 1.0;
K = 1.0;
epsilon = 1e-1;

var = 2*(number_evader+1);
N = var*number_interval;
initial_point = rand(var,1)*2-1;
initial_point(var-1:var) = [-2;0];

lower_bound(1:N,1) = -100;
upper_bound(1:N,1) = 100;

% file = matfile('savestartingpoint.mat');
% starting_point = file.a;
starting_point = rand(N,1)*2-1;
% for t=1:number_interval
%     for i=1:var-2
%         if mod(i,2) ~= 0
%             starting_point(var*(t-1)+i) = 1+rand(1)*4;
%         else
%             starting_point(var*(t-1)+i) = rand(1)*3;
%         end
%     end
%     starting_point(var*t-1) = rand(1)*4;
%     starting_point(var*t) = rand(1)*4;
% end

% % ----------plotting the initial positions of the evader and pursuer
% fig = figure();
% ipx = zeros(number_evader,1);
% ipy = zeros(number_evader,1);
% for i=1:number_evader
%     ipx(i,1) = initial_point(i*2-1);
%     ipy(i,1) = initial_point(i*2);
% end
% plot(ipx, ipy, 'o', 'color', 'red');hold on;
% % plot(initial_point(var-1), initial_point(var), 'o', 'color', 'blue');hold on;
% % plot(0, 10, 'X', 'color', 'black');
% draw_circle(3, 3, 0.5);
% grid on;
% xlabel('X');
% ylabel('Y');
% xlim([-5,5]);
% ylim([-5, 5]);
% % legend('evaders','pursuer','destination');
% title('Aggregated Evaders');
% saveas(fig, '../../presentation/images/evader_desired_position.png');
% hold off;
% % fig = figure();

% -------------------------------------------------

A = [];
b = [];
Aeq = [];
beq = [];

options = optimoptions('fmincon', 'Algorithm', 'interior-point', ...
'MaxFunEvals', 1000000, 'MaxIter', 10000, 'TolFun', 1e-1, 'TolCon', 1e-1, 'TolX', 1e-12, ...
'Display', 'iter', 'GradObj', 'off', 'DerivativeCheck','off', 'FinDiffType', 'central');

obj_func = @(x)objective_function(x, number_interval, time_interval, var, initial_point);
nonlinearcons = @(x)constraints(x, number_interval, time_interval, number_evader, vemax_repulsion, vemax_attraction, vpmax, K, epsilon, var, initial_point);
[opt_x, fval, exitflag, output] = fmincon(obj_func, starting_point, A, b, Aeq, beq, lower_bound, upper_bound, nonlinearcons, options);

pursuer_x = zeros(number_interval+1,1);
pursuer_y = zeros(number_interval+1,1);
pursuer_x(1,1) = initial_point(var-1);
pursuer_y(1,1) = initial_point(var);

for t=1:number_interval
    pursuer_x(t+1,1) = opt_x(var*t-1);
    pursuer_y(t+1,1) = opt_x(var*t);
end
evader_final_x = zeros(number_evader,1);
evader_final_y = zeros(number_evader,1);
evader_initial_x = zeros(number_evader,1);
evader_initial_y = zeros(number_evader,1);
evader_path_x = zeros(number_interval-1,1);
evader_path_y = zeros(number_interval-1,1);

figure(1);
for i=1:number_evader
    evader_final_x(i,1) = opt_x(var*(number_interval-1)+2*i-1);
    evader_final_y(i,1) = opt_x(var*(number_interval-1)+2*i);
    evader_initial_x(i,1) = initial_point(2*i-1);
    evader_initial_y(i,1) = initial_point(2*i);

    for t=1:number_interval-1
        evader_path_x(t,1) = opt_x(var*(t-1)+2*i-1);
        evader_path_y(t,1) = opt_x(var*(t-1)+2*i);
    end
    plot(evader_path_x, evader_path_y, 'o-', 'color', 'magenta'); hold on;
end

final_centroid = [mean(evader_final_x);mean(evader_final_y)];

plot(pursuer_x, pursuer_y, 'o-', 'color', 'blue');hold on;
plot(evader_initial_x, evader_initial_y, 'o', 'color', 'red');hold on;
plot(evader_final_x, evader_final_y, 'o', 'color', 'green');hold on;
draw_circle(final_centroid(1), final_centroid(2), epsilon);
grid on;
xlabel('X');
ylabel('Y');
title('pursuer-evader-aggregation');
hold off;

evader_path_verify(number_evader, initial_point, number_interval, time_interval, opt_x, vemax_repulsion, vemax_attraction, K, var, pursuer_x, pursuer_y, epsilon);