function [ceq] = non_linear_equality_1(x, t, v, var, initial_point, number_interval)
    N = var*number_interval+1;
    if t == 1
        ceq = x(var*(t-1)+1) - initial_point(1) - v*cos(initial_point(3))*x(N);
    else
        ceq = x(var*(t-1)+1) - x(var*(t-2)+1) - v*cos(x(var*(t-2)+3))*x(N);
    end
end