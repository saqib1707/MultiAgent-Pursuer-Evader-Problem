function [cineq] = inequality_constraints(x, var, Ni, Ne, ti, iep, ipp, vemax_repulsion, vemax_attraction, epsilon, K, fd)
    parameters = horzcat(ipp,reshape(x,var,Ni));
    pursuer_position = parameters(1:end-1,:);
    evader_position = compute_evader_position(pursuer_position,Ne,iep,Ni,ti,vemax_repulsion,vemax_attraction,K);
    cineq(1:Ne,1) = sqrt(sum((reshape(evader_position(:,Ni+1),2,Ne) - repmat(fd,[1,Ne])).^2,1)) - epsilon;
%     cieq(Ne+1:Ne+Ni,1) = sqrt(sum((pursuer_position(:,2:Ni+1) - pursuer_position(:,1:Ni)).^2,1)) - vpmax*ti;
%     cieq(Ne+Ni+1:Ne+2*Ni,1) = vpmin*ti - sqrt(sum((pursuer_position(:,2:Ni+1) - pursuer_position(:,1:Ni)).^2,1));
end