function [ceq] = equality_constraints(x, var, Ni, ti, ipp)
    parameters = horzcat(ipp,reshape(x,var,Ni));
    pursuer_position = parameters(1:end-1,:);
    pursuer_velocity = parameters(end,:);
    ceq(1:Ni,1) = sqrt(sum((pursuer_position(:,2:Ni+1) - pursuer_position(:,1:Ni)).^2,1)) - pursuer_velocity(:,2:Ni+1)*ti;
end