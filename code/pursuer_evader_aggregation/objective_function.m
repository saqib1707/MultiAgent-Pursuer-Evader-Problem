function [path_length] = objective_function(x,Ni,var,ipp)
    parameters = horzcat(ipp,reshape(x,var,Ni));
    temp = parameters(:,2:Ni+1) - parameters(:,1:Ni);
    path_length = sum(sqrt(sum(temp.^2,1)));
end