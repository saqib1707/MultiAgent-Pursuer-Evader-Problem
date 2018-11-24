function [ceq] = non_linear_equality_3(x, t, var, initial_point, number_interval)
    N = var*number_interval+1;
    if t == 1
        ceq = x(var*(t-1)+3) - initial_point(3) - initial_point(4)*x(N);
    else
        ceq = x(var*(t-1)+3) - x(var*(t-2)+3) - x(var*(t-2)+4)*x(N);
    end
end