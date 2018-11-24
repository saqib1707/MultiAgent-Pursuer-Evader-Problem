clear; clc;

number_interval = 100;
v = 1.0;
input_bound = 1.0;
epsilon = 1e-2;
var = 4;
N = var*number_interval+1;
initial_point = [4.0, 4.0, 45*pi/180.0, 0.5];

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
    if(mod(i, var) == var-1)
        lower_bound(i,1) = 0;
        upper_bound(i,1) = 2*pi;
        starting_point(i,1) = rand(1)*2*pi;
    end;
end;
lower_bound(N,1) = 0;
upper_bound(N,1) = inf;
starting_point(N,1) = 5.0;

A = [];
b = [];
Aeq = [];
beq = [];

options = optimoptions(@fmincon, 'MaxFunEvals', 100000);
nonlcons = @(x)constraints(x, number_interval, epsilon, var, v, initial_point);
opt_x = fmincon(obj_fun, starting_point, A, b, Aeq, beq, lower_bound, upper_bound, nonlcons, options);

x_coord = [initial_point(1)];
y_coord = [initial_point(2)];

for i=1:number_interval
    x_coord(i+1,1) = opt_x(var*(i-1)+1);
    y_coord(i+1,1) = opt_x(var*(i-1)+2);
end

plot(x_coord, y_coord);
% Label the plot 
xlabel('X'); 
ylabel('Y'); 
title('dubins vehicle problem');  
grid on;
hold on;
draw_circle(0,0,sqrt(epsilon));