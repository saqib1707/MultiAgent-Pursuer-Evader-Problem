function [cieq] = inequality_constraints(x, var, Ni, Ne, ti, iep, ipp, vemax_repulsion, vemax_attraction, vpmax, vpmin, epsilon, K, fd)
    pursuer_position = horzcat(ipp,reshape(x,var,Ni));
    evader_position = compute_evader_position(pursuer_position,Ne,iep,Ni,ti,vemax_repulsion,vemax_attraction,K);
    cieq(1:Ne,1) = sqrt(sum((reshape(evader_position(:,Ni+1),2,Ne) - repmat(fd,[1,Ne])).^2,1)) - epsilon;
    cieq(Ne+1:Ne+Ni,1) = sqrt(sum((pursuer_position(:,2:Ni+1) - pursuer_position(:,1:Ni)).^2,1)) - vpmax*ti;
    cieq(Ne+Ni+1:Ne+2*Ni,1) = vpmin*ti - sqrt(sum((pursuer_position(:,2:Ni+1) - pursuer_position(:,1:Ni)).^2,1));
end