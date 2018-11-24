function [cieq, ceq] = constraints(x, Ni, ti, Ne, vemax, vpmax, K, epsilon, var, ip)
    cieq = non_linear_inequality_1(x, var, Ni, Ne, ti, ip, vpmax, epsilon);
    ceq = non_linear_equality_1(x, vemax, Ne, Ni, var, K, ip, ti);
end