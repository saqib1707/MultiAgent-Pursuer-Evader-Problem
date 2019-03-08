function [path_length] = objective_function(x, Ni, var, ip)
    parameters = horzcat(ip,reshape(x,var,Ni));
    temp = parameters(var-1:var,2:Ni+1) - parameters(var-1:var,1:Ni);
    path_length = sum(sqrt(sum(temp.^2,1)));
end