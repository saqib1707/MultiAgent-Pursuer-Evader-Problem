function [c, ceq] = constraints(x, number_interval, epsilon, var, initial_point)
    count = 0;
    c = x(var*number_interval-2)^2 + x(var*number_interval-1)^2 - epsilon;
    for t=1:number_interval
        count = count + 1;
        ceq(count) = non_linear_equality_1(x, t, initial_point, number_interval, var);
        count = count + 1;
        ceq(count) = non_linear_equality_2(x, t, initial_point, number_interval, var);
    end
end