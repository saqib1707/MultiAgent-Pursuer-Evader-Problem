function [cieq, ceq] = non_linear_constraints(x,var,Ni,Ne,ti,iep,ipp,vemax_repulsion,vemax_attraction,vpmax,vpmin,epsilon,K,fd)  
    cieq = inequality_constraints(x,var,Ni,Ne,ti,iep,ipp,vemax_repulsion,vemax_attraction,vpmax,vpmin,epsilon,K,fd);
    ceq = [];
end