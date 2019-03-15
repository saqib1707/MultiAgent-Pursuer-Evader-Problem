function [cineq, ceq] = non_linear_constraints(x,var,Ni,Ne,ti,iep,ipp,vemax_repulsion,vemax_attraction,vpmax,vpmin,epsilon,K,fd)  
    cineq = inequality_constraints(x,var,Ni,Ne,ti,iep,ipp,vemax_repulsion,vemax_attraction,epsilon,K,fd);
    ceq = equality_constraints(x, var, Ni, ti, ipp);
end