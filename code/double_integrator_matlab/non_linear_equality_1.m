function [ceq] = non_linear_equality_1(x, t, initial_point, number_interval, var)
    N = var*number_interval+1;
    if t == 1
        ceq = x(var*(t-1)+1) - initial_point(1) - initial_point(2)*x(N);
    else
        ceq = x(var*(t-1)+1) - x(var*(t-2)+1) - x(var*(t-2)+2)*x(N);
    end
end