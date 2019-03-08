function [cieq, ceq] = constraints(x, Ni, ti, Ne, vemax_repulsion, vemax_attraction, vpmax, vpmin, K, epsilon, var, ip, fd)
    cieq = non_linear_inequality(x, var, Ni, Ne, ti, ip, vpmax, vpmin, epsilon, fd);
    ceq = non_linear_equality(x, vemax_repulsion, vemax_attraction, Ne, Ni, var, K, ip, ti);
end