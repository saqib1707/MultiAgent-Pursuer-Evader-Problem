function [cieq, ceq] = constraints(x, Ni, ti, Ne, vemax, vemax2, vpmax, K, epsilon, var, ip)
    cieq = non_linear_inequality(x, var, Ni, Ne, ti, ip, vpmax, epsilon);
    ceq = non_linear_equality(x, vemax, vemax2, Ne, Ni, var, K, ip, ti);
end