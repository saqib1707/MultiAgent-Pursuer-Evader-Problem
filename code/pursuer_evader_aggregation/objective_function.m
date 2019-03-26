function [path_length] = objective_function(x, Ni, var, ipp)
    parameters = horzcat(ipp,reshape(x,var,Ni));
    temp = parameters(:,2:Ni+1) - parameters(:,1:Ni);
    path_length = sum(sqrt(sum(temp.^2,1)));
    
    % computing the gradients of objective function
    if nargout > 1
        dJ_dx = zeros(size(x));
        distance = sqrt(sum(temp.^2,1));
        for index = 1:Ni-1
           dJ_dx((index-1)*2+1:index*2,1) = (parameters(:,index+1) - parameters(:,index))/distance(index) ...
               - (parameters(:,index+2) - parameters(:,index+1))/distance(index+1);
        end
        dJ_dx(end-1:end,1) = (parameters(:,end) - parameters(:,end-1))/distance(end);
    end
end