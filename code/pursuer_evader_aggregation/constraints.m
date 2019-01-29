function [cieq, ceq] = constraints(x, Ni, ti, Ne, vemax_repulsion, vemax_attraction, vpmax, K, epsilon, var, ip, dth)
    cieq = non_linear_inequality(x, var, Ni, Ne, ti, ip, vpmax, epsilon);
    ceq = non_linear_equality(x, vemax_repulsion, vemax_attraction, Ne, Ni, var, K, ip, ti, dth);
end