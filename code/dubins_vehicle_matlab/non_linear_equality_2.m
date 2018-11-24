function [ceq] = non_linear_equality_2(x, t, v, var, initial_point, number_interval)
    N = var*number_interval+1;
    if t == 1
        ceq = x(var*(t-1)+2) - initial_point(2) - v*sin(initial_point(3))*x(N);
    else
        ceq = x(var*(t-1)+2) - x(var*(t-2)+2) - v*sin(x(var*(t-2)+3))*x(N);
    end
end