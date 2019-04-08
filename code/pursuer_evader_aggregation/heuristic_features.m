clear;clc;
root_dir = '../../results-plots/19-03-19/';
for index=1:43
    filename = strcat(strcat(root_dir,'hyperparameters/'),strcat(int2str(index),'.mat'));
    load(filename);
    pursuer_trajectory = horzcat(hp.initial_pursuer_position,reshape(hp.opt_x,2,hp.number_interval));
    evader_trajectory = compute_evader_position(pursuer_trajectory,hp.number_evader,hp.initial_evader_position,hp.number_interval,hp.time_interval,hp.vemax_repulsion,hp.vemax_attraction,hp.K);
    evader_center_trajectory = (evader_trajectory(1:2,:)+evader_trajectory(3:4,:))/2;
    pursuer_velocity = pursuer_trajectory(:,2:end) - pursuer_trajectory(:,1:end-1);
    pursuer_evader_center_vector = evader_center_trajectory(:,1:end-1) - pursuer_trajectory(:,1:end-1);

    figure;
    plot(pursuer_trajectory(1,:), pursuer_trajectory(2,:), 'o-', 'color', 'blue');hold on;
    for i=1:hp.number_evader
        plot(evader_trajectory(2*i-1,1),evader_trajectory(2*i,1), 'o-', 'color', 'green');hold on;
        plot(evader_trajectory(2*i-1,2:hp.number_interval),evader_trajectory(2*i,2:hp.number_interval),'o-','color','yellow');hold on;
        plot(evader_trajectory(2*i-1,hp.number_interval+1),evader_trajectory(2*i,hp.number_interval+1), 'o-', 'color', 'red');hold on;
    end
    for t = 1:hp.number_interval
         plot([evader_trajectory(1,t),evader_trajectory(3,t)],[evader_trajectory(2,t),evader_trajectory(4,t)],'red');
    %     plot([evader_center_trajectory(1,t),pursuer_trajectory(1,t)],[evader_center_trajectory(2,t),pursuer_trajectory(2,t)],'black');
    %     plot([evader_center_trajectory(1,t),hp.destination(1,1)],[evader_center_trajectory(2,t),hp.destination(2,1)],'red');
    %     plot([evader_trajectory(1,t),pursuer_trajectory(1,t)],[evader_trajectory(2,t),pursuer_trajectory(2,t)],'green');
    %     plot([evader_trajectory(3,t),pursuer_trajectory(1,t)],[evader_trajectory(4,t),pursuer_trajectory(2,t)],'red');
    end
    draw_circle(hp.destination(1,1), hp.destination(2,1), hp.epsilon);
    grid on;
    xlabel('X');
    ylabel('Y');
    title('shepherding-optimization-result');
    hold off;

    % angle between the pursuer-evader_center and destination-evader_center
    % theta = zeros(hp.number_interval,1);
    % for t = 1:hp.number_interval
    %     a = pursuer_trajectory(:,t) - evader_center_trajectory(:,t);
    %     b = hp.destination - evader_center_trajectory(:,t);
    %     theta(t,1) = acos(dot(a,b)/(norm(a)*norm(b)))*(180/pi);
    % end
    % 
    % figure;
    % plot(theta, 'o-', 'color', 'blue');hold on;
    % grid on;
    % xlabel('time step');
    % ylabel('angle (in degrees)');
    % title('angle between pursuer\_position and destination wrt evader\_center_trajectory');
    % hold off;

    % plotting the evader-separation over time 
%     evader_separation = sqrt(sum((evader_trajectory(1:2,:) - evader_trajectory(3:4,:)).^2,1));
%     h1 = figure;
%     plot(evader_separation, 'o-', 'color', 'blue');hold on;
%     grid on;
%     xlabel('time step');
%     ylabel('evader separation');
%     title('separation between evaders');
%     hold off;
%     savefilename = strcat(strcat(root_dir,'feature_1/'),strcat(int2str(index),'.fig'));
%     savefig(h1, savefilename);

    % plotting the angle of the evader-vector measured wrt X-axis
%     evader_vector_slope = (evader_trajectory(4,:) - evader_trajectory(2,:))./(evader_trajectory(3,:) - evader_trajectory(1,:));
%     evader_vector_angle = atan(evader_vector_slope)*180/pi;
%     % evader_vector_angle(evader_vector_angle < 0) = 180 + evader_vector_angle(evader_vector_angle < 0);
%     h2 = figure;
%     plot(evader_vector_angle, 'o-', 'color', 'blue');hold on;
%     grid on;
%     xlabel('time step');
%     ylabel('evader vector slope (in degrees)');
%     title('slope of line joining evaders');
%     hold off;
%     savefilename = strcat(strcat(root_dir,'feature_2/'),strcat(int2str(index),'.fig'));
%     savefig(h2, savefilename);

%     checking whether the two triangles are similar
%     pursuer_evader_vector = evader_trajectory - repmat(pursuer_trajectory,[hp.number_evader,1]);
%     evader_evader_vector = evader_trajectory(1:2,:) - evader_trajectory(3:4,:);
%     pursuer_evader_distance = [sqrt(sum(pursuer_evader_vector(1:2,:).^2,1));sqrt(sum(pursuer_evader_vector(3:4,:).^2,1))];
%     evader_evader_distance = sqrt(sum(evader_evader_vector.^2,1));
%     similarity_ratio = [pursuer_evader_distance(:,2:hp.number_interval+1)./pursuer_evader_distance(:,1:hp.number_interval);...
%         evader_evader_distance(:,2:hp.number_interval+1)./evader_evader_distance(:,1:hp.number_interval)];

    % plotting the angle between pursuer-velocity and evader-center-velocity vector
%     evader_center_velocity = evader_center_trajectory(:,2:end) - evader_center_trajectory(:,1:end-1);
%     costheta = zeros(hp.number_interval,1);
%     for t = 1:hp.number_interval
%         costheta(t,1) = dot(pursuer_velocity(:,t),evader_center_velocity(:,t))/(norm(pursuer_velocity(:,t))*norm(evader_center_velocity(:,t)));
%     end
%     theta = acos(costheta)*180/pi;
%     h3 = figure;
%     plot(theta, 'o-', 'color', 'blue');hold on;
%     grid on;
%     xlabel('time step');
%     ylabel('angle (in degrees)');
%     title('angle between pursuer\_velocity and evader\_center\_velocity');
%     hold off;
%     savefilename = strcat(strcat(root_dir,'feature_3/'),strcat(int2str(index),'.fig'));
%     savefig(h3, savefilename);

    % plotting the angle between pursuer-velocity and destination-evader_center vector
%     dest_evader_center_vector = repmat(hp.destination,[1,hp.number_interval]) - evader_center_trajectory(:,1:end-1);
%     costheta = zeros(hp.number_interval,1);
%     for t = 1:hp.number_interval
%         costheta(t,1) = dot(pursuer_velocity(:,t),dest_evader_center_vector(:,t))/(norm(pursuer_velocity(:,t))*norm(dest_evader_center_vector(:,t)));
%     end
%     theta = acos(costheta)*180/pi;
%     h4 = figure;
%     plot(theta, 'o-', 'color', 'blue');hold on;
%     grid on;
%     xlabel('time step');
%     ylabel('angle (in degrees)');
%     title('angle between pursuer\_velocity and destination\_evader\_center\_vector');
%     hold off;
%     savefilename = strcat(strcat(root_dir,'feature_4/'),strcat(int2str(index),'.fig'));
%     savefig(h4, savefilename);

    % plotting the angle between pursuer_velocity and pursuer-evader_center
    costheta = zeros(hp.number_interval,1);
    for t = 1:hp.number_interval
        costheta(t,1) = dot(pursuer_velocity(:,t),pursuer_evader_center_vector(:,t))/(norm(pursuer_velocity(:,t))*norm(pursuer_evader_center_vector(:,t)));
    end
    theta = acos(costheta)*180/pi;
    h5 = figure;
    plot(theta, 'o-', 'color', 'blue');hold on;
    grid on;
    xlabel('time step');
    ylabel('angle (in degrees)');
    title('angle between pursuer\_velocity and pursuer\_evader\_center\_vector');
    hold off;
    savefilename = strcat(strcat(root_dir,'feature_5/'),strcat(int2str(index),'.fig'));
    savefig(h5, savefilename);

    close all;
end