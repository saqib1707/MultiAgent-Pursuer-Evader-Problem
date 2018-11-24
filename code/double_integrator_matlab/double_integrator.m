clear;clc;

number_interval = 60;
epsilon = 1e-2;
input_bound = 1.0;
var = 3;
N = var*number_interval+1;
initial_point = [4.0, 4.0, 0.5];

obj_fun = @(x)number_interval*x(N);

lower_bound = -inf(N,1);
upper_bound = inf(N,1);
starting_point = rand(N,1)*8 - 4;
for i=1:N-1
    if(mod(i, var) == 0)
        lower_bound(i,1) = -input_bound;
        upper_bound(i,1) = input_bound;
        starting_point(i,1) = rand(1)*2 - 1;
    end;
end;
lower_bound(N,1) = 0;
upper_bound(N,1) = inf;
starting_point(N,1) = 5.0;

A = [];
b = [];
Aeq = [];
beq = [];

options = optimoptions(@fmincon, 'MaxFunEvals', 30000);
nonlcons = @(x)constraints(x, number_interval, epsilon, var, initial_point);
opt_x = fmincon(obj_fun, starting_point, A, b, Aeq, beq, lower_bound, upper_bound, nonlcons, options);

x_coord = [initial_point(1)];
y_coord = [initial_point(2)];

for i=1:number_interval
    x_coord(i+1,1) = opt_x(var*i-2);
    y_coord(i+1,1) = opt_x(var*i-1);
end

plot(x_coord, y_coord, 'o-'); 
xlabel('X'); 
ylabel('Y'); 
title('double integrator system'); 
grid on;
hold on;
draw_circle(0,0,sqrt(epsilon));