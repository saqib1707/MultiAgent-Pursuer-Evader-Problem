function [c, ceq] = constraints(x, number_interval, epsilon, var, v, initial_point)
    count = 0;
 
    c = x(var*(number_interval-1)+1)^2 + x(var*(number_interval-1)+2)^2 - epsilon;
    for t=1:number_interval
        count = count + 1;
        ceq(count) = non_linear_equality_1(x, t, v, var, initial_point, number_interval);
        count = count + 1;
        ceq(count) = non_linear_equality_2(x, t, v, var, initial_point, number_interval);
        count = count + 1;
        ceq(count) = non_linear_equality_3(x, t, var, initial_point, number_interval);
    end
end