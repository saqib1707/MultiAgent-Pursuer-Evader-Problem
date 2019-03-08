function [ceq] = non_linear_equality(x, vemax_repulsion, vemax_attraction, Ne, Ni, var, K, ip, ti)
    parameters = horzcat(ip,reshape(x,var,Ni));
    for t=1:Ni
        pursuer_evader_vector = reshape(parameters(1:Ne*2,t),2,Ne) - repmat(parameters(var-1:var,t),[1,Ne]);
        pursuer_velocity = (parameters(var-1:var,t+1) - parameters(var-1:var,t))/ti;
        pursuer_evader_distance = sqrt(sum(pursuer_evader_vector.^2,1));
        costheta = (transpose(pursuer_velocity)*pursuer_evader_vector)./(pursuer_evader_distance*norm(pursuer_velocity));
        evader_position_sum = sum(reshape(parameters(1:Ne*2,t),2,Ne),2);
        centroid_nearest_neighbour = (repmat(evader_position_sum,[1,Ne]) - reshape(parameters(1:Ne*2,t),2,Ne))/(Ne-1);
        centroid_evader_vector = centroid_nearest_neighbour - reshape(parameters(1:Ne*2,t),2,Ne);
        centroid_evader_distance = sqrt(sum(centroid_evader_vector.^2,1));
        repulsion_term = reshape(0.5*vemax_repulsion*repmat(exp(-K*pursuer_evader_distance).*(1+costheta),[2,1]).*(pursuer_evader_vector./repmat(pursuer_evader_distance,[2,1])),Ne*2,1);
        attraction_term = reshape(vemax_attraction*repmat(exp(-K*pursuer_evader_distance),[2,1]).*(centroid_evader_vector./repmat(centroid_evader_distance,[2,1])),Ne*2,1);
        ceq(Ne*2*(t-1)+1:Ne*2*t,1) = parameters(1:Ne*2,t+1) - parameters(1:Ne*2,t) - repulsion_term*ti - attraction_term*ti;
    end
end