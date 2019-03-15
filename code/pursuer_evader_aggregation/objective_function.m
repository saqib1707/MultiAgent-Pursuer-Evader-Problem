function [path_length] = objective_function(x, Ni, var, ipp)
    parameters = horzcat(ipp,reshape(x,var,Ni));
    pursuer_position = parameters(1:end-1,:);
    temp = pursuer_position(:,2:Ni+1) - pursuer_position(:,1:Ni);
    path_length = sum(sqrt(sum(temp.^2,1)));
end