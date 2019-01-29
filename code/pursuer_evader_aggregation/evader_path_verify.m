function [error] = evader_path_verify(Ne, ip, Ni, ti, opt_x, vemax_repulsion, vemax_attraction, K, var, epsilon, dth)

    % Below part is just for verifying whether evaders and pursuer
    % is following the control law
    % Reconstructing the path of the evader based on the pursuer path obtained
    % from fmincon
    parameters = horzcat(ip,reshape(opt_x,var,Ni));
    pursuer_position = parameters(var-1:var,:);
    evader_position = zeros(Ne*2,Ni+1);
    evader_position(:,1) = ip(1:Ne*2,1);

    for i=1:Ni
        pursuer_evader_vector = reshape(evader_position(:,i),2,Ne) - repmat(pursuer_position(:,i),[1,Ne]);
        pursuer_velocity = pursuer_position(:,i+1) - pursuer_position(:,i);
        pursuer_evader_distance = sqrt(sum(pursuer_evader_vector.^2,1));
        costheta = (transpose(pursuer_velocity)*pursuer_evader_vector)./(pursuer_evader_distance*norm(pursuer_velocity));
        evader_position_sum = sum(reshape(evader_position(:,i),2,Ne),2);
        centroid_nearest_neighbour = (repmat(evader_position_sum,[1,Ne]) - reshape(evader_position(:,i),2,Ne))/(Ne-1);
        centroid_evader_vector = centroid_nearest_neighbour - reshape(evader_position(:,i),2,Ne);
        centroid_evader_distance = sqrt(sum(centroid_evader_vector.^2,1));
%         d_factor = (centroid_evader_distance <= dth).*(centroid_evader_distance/dth) + (centroid_evader_distance > dth).*(1);
        repulsion_term = reshape(0.5*vemax_repulsion*repmat(exp(-K*pursuer_evader_distance).*(1+costheta),[2,1]).*(pursuer_evader_vector./repmat(pursuer_evader_distance,[2,1])),2*Ne,1);
        attraction_term = reshape(vemax_attraction*repmat(exp(-K*pursuer_evader_distance),[2,1]).*(centroid_evader_vector./repmat(centroid_evader_distance,[2,1])),2*Ne,1);
        evader_position(:,i+1) = evader_position(:,i) + repulsion_term*ti + attraction_term*ti;
    end
%     centroid = zeros(2,Ni);
%     ep = zeros(2*Ne, Ni+1);
%     ep(:,1) = ip(1:Ne*2);
%     centroid(:,1) = sum(reshape(ep(:,1),2,Ne),2);

%     for i = 1:Ne
%         temp = (centroid(:,1) - ep(2*i-1:2*i,1))/(Ne-1) - ep(2*i-1:2*i,1);
%         temp = temp/norm(temp);
%         ep(2*i-1:2*i,2) = compute_enc(ep(2*i-1:2*i,1), ip(var-1:var), opt_x(var-1:var), vemax, vemax2, ti, K, temp);
%     end
% 
%     for t=2:Ni
%         centroid(:,t) = sum(reshape(ep(:,t),2,Ne),2);
%         for i=1:Ne
%             temp = (centroid(:,t) - ep(2*i-1:2*i,t))/(Ne-1) - ep(2*i-1:2*i,t);
%             temp = temp/norm(temp);
%             ep(2*i-1:2*i,t+1) = compute_enc(ep(2*i-1:2*i,t), opt_x(var*(t-1)-1:var*(t-1)), opt_x(var*t-1:var*t), vemax, vemax2, ti, K, temp);
%         end
%     end

    evader_position_gt = parameters(1:Ne*2,:);   % which is obtained from optimization
    error = norm(evader_position - evader_position_gt,'fro');

    figure;
    plot(pursuer_position(1,:),pursuer_position(2,:), 'o-', 'color', 'blue');hold on;
    for i=1:Ne
        plot(evader_position(2*i-1,1),evader_position(2*i,1), 'o-', 'color', 'green');hold on;
        plot(evader_position(2*i-1,2:Ni),evader_position(2*i,2:Ni), 'o-', 'color', 'yellow');hold on;
        plot(evader_position(2*i-1,Ni+1),evader_position(2*i,Ni+1), 'o-', 'color', 'red');hold on;
    end
    final_centroid = mean(reshape(evader_position(:,Ni+1),2,Ne),2);
    draw_circle(final_centroid(1,1),final_centroid(2,1),epsilon);
    grid on;
    xlabel('X');
    ylabel('Y');
    title('shepherding-ground-truth');
    hold off;

%     final_centroid = sum(reshape(ep(:,Ni+1),2,Ne),2)/Ne;
%     figure(2);
%     for i=1:Ne
%         plot(ep(2*i-1,1),ep(2*i,1), 'o-', 'color', 'red');hold on;
%         plot(ep(2*i-1,2:Ni),ep(2*i,2:Ni), 'o-', 'color', 'magenta');hold on;
%         plot(ep(2*i-1,Ni+1),ep(2*i,Ni+1), 'o-', 'color', 'green');hold on;
%     end
%     plot(pursuer_x, pursuer_y, 'o-', 'color', 'blue');hold on;
%     draw_circle(final_centroid(1), final_centroid(2), epsilon);
%     grid on;
%     xlabel('X');
%     ylabel('Y');
%     title('pursuer-evader-aggregation-gt-check');
%     hold off;
end