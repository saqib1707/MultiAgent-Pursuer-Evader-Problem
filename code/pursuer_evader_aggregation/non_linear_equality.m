function [ceq] = non_linear_equality(x, vemax_repulsion, vemax_attraction, Ne, Ni, var, K, ip, ti, dth)
%     count = 0;
    parameters = horzcat(ip, reshape(x,var,Ni));
    for t=1:Ni
        pursuer_evader_vector = reshape(parameters(1:Ne*2,t),2,Ne) - repmat(parameters(var-1:var,t),[1,Ne]);
        pursuer_velocity = parameters(var-1:var,t+1) - parameters(var-1:var,t);
        pursuer_evader_distance = sqrt(sum(pursuer_evader_vector.^2,1));
        costheta = (transpose(pursuer_velocity)*pursuer_evader_vector)./(pursuer_evader_distance*norm(pursuer_velocity));
        evader_position_sum = sum(reshape(parameters(1:Ne*2,t),2,Ne),2);
        centroid_nearest_neighbour = (repmat(evader_position_sum,[1,Ne]) - reshape(parameters(1:Ne*2,t),2,Ne))/(Ne-1);
        centroid_evader_vector = centroid_nearest_neighbour - reshape(parameters(1:Ne*2,t),2,Ne);
        centroid_evader_distance = sqrt(sum(centroid_evader_vector.^2,1));
%         d_factor = (centroid_evader_distance <= dth).*(centroid_evader_distance/dth) + (centroid_evader_distance > dth).*(1);
        repulsion_term = reshape(0.5*vemax_repulsion*repmat(exp(-K*pursuer_evader_distance).*(1+costheta),[2,1]).*(pursuer_evader_vector./repmat(pursuer_evader_distance,[2,1])),Ne*2,1);
        attraction_term = reshape(vemax_attraction*repmat(exp(-K*pursuer_evader_distance),[2,1]).*(centroid_evader_vector./repmat(centroid_evader_distance,[2,1])),Ne*2,1);
        ceq(Ne*2*(t-1)+1:Ne*2*t,1) = parameters(1:Ne*2,t+1) - parameters(1:Ne*2,t) - repulsion_term*ti - attraction_term*ti;
    end
%     centroid = zeros(2,Ni);
%     for t = 0:Ni-1
%         if t == 0
%             centroid(:,t+1) = sum(reshape(ip(1:Ne*2),2,Ne),2);
%         else
%             centroid(:,t+1) = sum(reshape(x(var*(t-1)+1:var*(t-1)+Ne*2),2,Ne),2);
%         end
%     end
    
%     for i = 1:Ne
%         for t = 0:Ni-1
%             if t == 0
%                 losv = ip(2*i-1:2*i) - ip(var-1:var);
%                 term1 = (x(2*i-1:2*i) - ip(2*i-1:2*i))/ti;
%                 term3 = x(var-1:var) - ip(var-1:var);
%                 nn_centroid = (centroid(:,t+1) - ip(2*i-1:2*i))/(Ne-1);
%                 sec_direction = nn_centroid - ip(2*i-1:2*i);
%             else
%                 losv = x(var*(t-1)+2*i-1:var*(t-1)+2*i) - x(var*(t-1)+var-1:var*(t-1)+var);
%                 term1 = (x(var*t+2*i-1:var*t+2*i) - x(var*(t-1)+2*i-1:var*(t-1)+2*i))/ti;
%                 term3 = x(var*t+var-1:var*t+var) - x(var*(t-1)+var-1:var*(t-1)+var);
%                 nn_centroid = (centroid(:,t+1) - x(var*(t-1)+2*i-1:var*(t-1)+2*i))/(Ne-1);
%                 sec_direction = nn_centroid - x(var*(t-1)+2*i-1:var*(t-1)+2*i);
%             end
% 
%             sec_direction = sec_direction/norm(sec_direction);
%             distance = norm(losv);
%             term2 = exp(-K*distance);
%             costheta = dot(losv,term3)/(norm(term3)*distance);
% 
%             for z=1:2
%                 count = count + 1;
%                 ceq(count) = term1(z) - vemax*0.5*term2*(1+costheta)*losv(z)/distance - vemax2*term2*sec_direction(z);
%             end
%         end
%     end
end