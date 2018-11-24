function [ceq] = non_linear_equality_2(x, t, initial_point, number_interval, var)
    N = var*number_interval+1;
    if t == 1
        ceq = x(var*(t-1)+2) - initial_point(2) - initial_point(3)*x(N);
    else
		ceq = x(var*(t-1)+2) - x(var*(t-2)+2) - x(var*(t-2)+3)*x(N);
    end
end