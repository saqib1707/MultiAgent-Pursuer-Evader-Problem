function [path_length] = objective_function(x, Ni, ti, var, ip)
    path_length = 0;
    for t = 1:Ni
        if t == 1
            temp = x(var*t-1:var*t) - ip(var-1:var);
            path_length = path_length + transpose(temp)*temp;
        else
            temp = x(var*t-1:var*t) - x(var*(t-1)-1:var*(t-1));
            path_length = path_length + transpose(temp)*temp;
        end
    end
    path_length = path_length/ti;
    
%     if nargout > 1
%         g = zeros(var*Ni,1);
% 
%         g(var-1:var) = 4*x(var-1:var) - 2*ip(var-1:var) - 2*x(2*var-1:2*var);
%         for t = 2:ti-1
%             g(var*t-1:var*t) = 4*x(var*t-1:var*t) - 2*x(var*(t-1)-1:var*(t-1)) - 2*x(var*(t+1)-1:var*(t+1));
%         end
%         g(var*Ni-1:var*Ni) = 2*x(var*Ni-1:var*Ni) - 2*x(var*(Ni-1)-1:var*(Ni-1));
%     end
end