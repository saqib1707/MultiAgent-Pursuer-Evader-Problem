function [cieq, ceq] = constraints(x,var,Ni,Ne,ti,iep,ipp,vemax_repulsion,vemax_attraction,vpmax,vpmin,epsilon,K,fd)  
    cieq = non_linear_inequality(x,var,Ni,Ne,ti,iep,ipp,vemax_repulsion,vemax_attraction,vpmax,vpmin,epsilon,K,fd);
    ceq = [];
end