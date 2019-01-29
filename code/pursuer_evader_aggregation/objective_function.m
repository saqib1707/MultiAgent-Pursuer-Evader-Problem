function [path_length] = objective_function(x, Ni, ti, var, ip)
    parameters = horzcat(ip,reshape(x,var,Ni));
    temp = parameters(var-1:var,2:Ni+1) - parameters(var-1:var,1:Ni);
    path_length = sum(sqrt(sum(temp.^2,1)));
%     path_length = 0;
%     for t = 1:Ni
%         if t == 1
%             temp = x(var*t-1:var*t) - ip(var-1:var);      % using backward differentiation
%         else
%             temp = x(var*t-1:var*t) - x(var*(t-1)-1:var*(t-1));    % using backward differentiation
%         end
%         path_length = path_length + norm(temp);
%     end
end