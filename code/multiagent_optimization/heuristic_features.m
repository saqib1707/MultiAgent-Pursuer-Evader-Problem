clc;clear;close all;
root_dir = '../../results-plots/19-03-19/hyperparameters/';
num_files = size(dir(root_dir),1);
pause_time = 0.1;

for index = 16
    filename = strcat(root_dir, int2str(index), '.mat');
    load(filename);
%     if not(and((hp.destination(1,1) > 0), (hp.destination(2,1) < 0)))
%         continue
%     end
    disp(['File Index :', num2str(index)]);
    pursuer_trajectory = horzcat(hp.initial_pursuer_position,reshape(hp.opt_x,2,hp.number_interval));
    evader_trajectory = compute_evader_position(pursuer_trajectory,hp.number_evader,hp.initial_evader_position,hp.number_interval,hp.time_interval,hp.vemax_repulsion,hp.vemax_attraction,hp.K);
    evader_center_trajectory = (evader_trajectory(1:2,:)+evader_trajectory(3:4,:))/2;
    pursuer_velocity = pursuer_trajectory(:,2:end) - pursuer_trajectory(:,1:end-1);
    pursuer_evader_center_vector = evader_center_trajectory(:,1:end) - pursuer_trajectory(:,1:end);

%     h0 = figure;
%     plot(pursuer_trajectory(1,:), pursuer_trajectory(2,:), '.-', 'color', 'blue');hold on;
%     for i=1:hp.number_evader
%         plot(evader_trajectory(2*i-1,1),evader_trajectory(2*i,1), '.-', 'color', 'green');hold on;
%         plot(evader_trajectory(2*i-1,2:hp.number_interval),evader_trajectory(2*i,2:hp.number_interval),'.-','color','yellow');hold on;
%         plot(evader_trajectory(2*i-1,hp.number_interval+1),evader_trajectory(2*i,hp.number_interval+1), '.-', 'color', 'red');hold on;
%     end


% ------------------- Visualization of pursuer-evader trajectories with time --------------------------
    h0 = figure;
    for t = 1:hp.number_interval+1
        if t == 1
            draw_circle(hp.destination(1,1), hp.destination(2,1), hp.epsilon);hold on;
        end
        plot([evader_trajectory(1,t),evader_trajectory(3,t)],[evader_trajectory(2,t),evader_trajectory(4,t)],'color','red','LineWidth',1);hold on;
%         plot([evader_center_trajectory(1,t),pursuer_trajectory(1,t)],[evader_center_trajectory(2,t),pursuer_trajectory(2,t)],'black');
%         plot([evader_center_trajectory(1,t),hp.destination(1,1)],[evader_center_trajectory(2,t),hp.destination(2,1)],'red');
%         plot([evader_trajectory(1,t),pursuer_trajectory(1,t)],[evader_trajectory(2,t),pursuer_trajectory(2,t)],'green');
%         plot([evader_trajectory(3,t),pursuer_trajectory(1,t)],[evader_trajectory(4,t),pursuer_trajectory(2,t)],'red');
        plot(pursuer_trajectory(1,t), pursuer_trajectory(2,t), '.', 'color', 'blue');hold on;
        grid on;
        xlabel('X');
        ylabel('Y');
        plot_title = strcat('shepherding-optimization-result', ' (iteration=', int2str(t), ')');
        title(plot_title);
        xlim([-3,3]);
        ylim([-3,3]);
        pause(pause_time);
%         clf(h0);
    end
    plot(pursuer_trajectory(1,:), pursuer_trajectory(2,:), '.-', 'color', 'blue');
    legend('destination', 'evader\_trajectory', 'pursuer\_trajectory');
    hold off;
    
%     savefilename_fig = strcat(root_dir, 'combined-main-plot-fig/', int2str(index), '.fig');
%     savefilename_png = strcat('../../btp-final-result/19-03-19-1/', int2str(index), '.png');
%     savefig(h0, savefilename_fig);
%     saveas(h0, savefilename_png);
% ----------------------------------------------------------------------------------------------


% ------------------------ Visualization of dipole direction with time ------------------------
    h1 = figure;
    collapsing_coord = hp.destination - pursuer_trajectory(:,end);
    for t = 1:hp.number_interval+1
        p1 = evader_trajectory(1:2,t) - evader_center_trajectory(:,t);
        p2 = evader_trajectory(3:4,t) - evader_center_trajectory(:,t);
        plot([p1(1),p2(1)],[p1(2),p2(2)],'color','red','LineWidth',1);hold on;
        plot([collapsing_coord(1,1),0],[collapsing_coord(2,1),0],'color','black');hold on;
        grid on;
        xlabel('X');
        ylabel('Y');
        plot_title = strcat('visualization of dipole direction', ' (iteration=', int2str(t), ')');
        title(plot_title);
        limit = norm(p1-p2);
        xlim([-limit/2,limit/2]);
        ylim([-limit/2,limit/2]);
        pause(pause_time);
        if t ~= hp.number_interval+1
            clf(h1);
        end
    end
    hold off;
% ----------------------------------------------------------------------------------------------


% ------------------------ Visualization of triangle collapsing to line ------------------------
    h2 = figure;
    limit = 3;
    for t = 1:hp.number_interval+1
        plot([hp.destination(1,1), pursuer_trajectory(1,t)], [hp.destination(2,1), pursuer_trajectory(2,t)], 'color', 'blue');hold on;
        plot([hp.destination(1,1), evader_center_trajectory(1,t)], [hp.destination(2,1), evader_center_trajectory(2,t)], 'color', 'blue');hold on;
        plot([pursuer_trajectory(1,t), evader_center_trajectory(1,t)], [pursuer_trajectory(2,t), evader_center_trajectory(2,t)], 'color', 'blue');hold on;
        plot([evader_trajectory(1,t), evader_trajectory(3,t)], [evader_trajectory(2,t), evader_trajectory(4,t)],'color','red');hold on;
        grid on;
        xlabel('X');
        ylabel('Y');
        plot_title = strcat('visualization of triangle collapse', ' (iteration=', int2str(t), ')');
        title(plot_title);
        xlim([-limit, limit]);
        ylim([-limit, limit]);
        pause(pause_time);
        if t ~= hp.number_interval+1
            clf(h2);
        end
    end
    centroid = (hp.destination + evader_center_trajectory(:,1) + pursuer_trajectory(:,1))/3;
    
    plot([hp.destination(1,1), pursuer_trajectory(1,1)], [hp.destination(2,1), pursuer_trajectory(2,1)], 'color', 'blue');hold on;
    plot([hp.destination(1,1), evader_center_trajectory(1,1)], [hp.destination(2,1), evader_center_trajectory(2,1)], 'color', 'blue');hold on;
    plot([pursuer_trajectory(1,1), evader_center_trajectory(1,1)], [pursuer_trajectory(2,1), evader_center_trajectory(2,1)], 'color', 'blue');hold on;
    plot([evader_trajectory(1,1), evader_trajectory(3,1)], [evader_trajectory(2,1), evader_trajectory(4,1)],'color','red');hold on;
    
    plot([centroid(1,1), hp.destination(1,1)], [centroid(2,1), hp.destination(2,1)], 'color', 'green');hold on;
    plot([centroid(1,1), pursuer_trajectory(1,1)], [centroid(2,1), pursuer_trajectory(2,1)], 'color', 'green');hold on;
    plot([centroid(1,1), evader_center_trajectory(1,1)], [centroid(2,1), evader_center_trajectory(2,1)], 'color', 'green');
    hold off;
% ----------------------------------------------------------------------------------------------


% ------------------------ Visualization of distance between pursuer & evader_center -------------------
%     h3 = figure;
%     for t = 1:hp.number_interval+1
%         distance = norm(pursuer_trajectory(:,t) - evader_center_trajectory(:,t));
%         plot(t, distance, '.', 'color', 'red');hold on;
%         grid on;
%         xlabel('X');
%         ylabel('Y');
%         plot_title = strcat('shepherding-optimization-result', ' (iteration=', int2str(t), ')');
%         title(plot_title);
%         xlim([1, hp.number_interval+1]);
%         ylim([0, 8]);
%         pause(pause_time);
%     end
%     plot(vecnorm(pursuer_trajectory - evader_center_trajectory), '.-', 'color', 'red');
%     hold off;
% ----------------------------------------------------------------------------------------------


% plots the evader center trajectory
%     plot(evader_center_trajectory(1,:), evader_center_trajectory(2,:), '.-', 'color', 'green');
%     draw_circle(hp.destination(1,1), hp.destination(2,1), hp.epsilon);


% ----------------------- identifying the circular and linear part of evader_center trajectory -----------------------
%     radius_threshold = 1.0;
%     min_reqd_points_for_circle = 3;
%     max_circles = hp.number_interval-min_reqd_points_for_circle+1;
%     points_list = zeros(3,2*max_circles);
%     for t = 1:max_circles
%         points_list(:,2*t-1:2*t) = transpose(evader_center_trajectory(:,t:t+min_reqd_points_for_circle-1));
%     end
%     [radius_list, center_list] = fit_circle_through_3_points(points_list);
%     loc = (radius_list <= radius_threshold);   % consider only those circles whose radius is smaller than radius_threshold
%     new_radius_list = radius_list(loc);
%     new_center_list = center_list(:,loc);
% 
%     num_circles = length(new_radius_list);
    
    % plotting the circles
%     for t = 1:num_circles
%         draw_circle(new_center_list(1,t), new_center_list(2,t), new_radius_list(1,t));
%     end

    % plotting the line joining center of consecutive circles
%     for t = 1:num_circles-1
%         plot([new_center_list(1,t+1), new_center_list(1,t)],[new_center_list(2,t+1), new_center_list(2,t)], '.-', 'color', 'cyan');
%     end

    % polynomial regression to fit the pursuer trajectory
%     poly_coeff = polyfit(pursuer_trajectory(1,:), pursuer_trajectory(2,:), 5);
%     x1 = pursuer_trajectory(1,:);
%     y1 = polyval(poly_coeff, x1);
%     norm_error_pursuer_path = norm(y1 - pursuer_trajectory(2,:));
%     plot(x1, y1, '.-', 'color', 'black');


% ----------------------- polynomial regression to fit the dipole center trajectory
%     poly_coeff = polyfit(evader_center_trajectory(1,:), evader_center_trajectory(2,:), 5);
%     x1 = evader_center_trajectory(1,:);
%     y1 = polyval(poly_coeff, x1);
%     norm_error_evader_center_path = norm(y1 - evader_center_trajectory(2,:));
%     plot(x1, y1, '.-', 'color', 'magenta');

%     grid on;
%     xlabel('X');
%     ylabel('Y');
%     plot_title = strcat('shepherding-optimization-result', ' (num\_circle=', int2str(num_circles), ')');
%     title(plot_title);
%     hold off;
    % savefilename_fig = strcat(root_dir, 'combined-main-plot-fig/', int2str(index), '.fig');
    % savefilename_png = strcat(root_dir, 'combined-main-plot-png/', int2str(index), '.png');
    % savefig(h0, savefilename_fig);
    % saveas(h0, savefilename_png);



%     h1 = figure;
%     radius_list(radius_list > radius_threshold) = 0;
%     subplot(2,2,1);plot(radius_list, '.-', 'color', 'blue');
%     grid on;
%     xlabel('circle\_index');
%     ylabel('radius of evader\_center circular path');
%     title('radius of the circles along the path');


% ----------------------- plotting the evader-separation over time -----------------------
%     evader_separation = sqrt(sum((evader_trajectory(1:2,:) - evader_trajectory(3:4,:)).^2,1));
%     subplot(2,2,2);plot(evader_separation, '.-', 'color', 'blue');
%     grid on;
%     xlabel('time step');
%     ylabel('evader separation');
%     title('separation between evaders');


% ----------------------- plotting the angle between pursuer-velocity and evader-center-velocity vector -----------------------
%     evader_center_velocity = evader_center_trajectory(:,2:end) - evader_center_trajectory(:,1:end-1);
%     costheta = zeros(hp.number_interval,1);
%     for t = 1:hp.number_interval
%         costheta(t,1) = dot(pursuer_velocity(:,t),evader_center_velocity(:,t))/(norm(pursuer_velocity(:,t))*norm(evader_center_velocity(:,t)));
%     end
%     theta = acos(costheta)*180/pi;
%     subplot(2,2,3);plot(theta, '.-', 'color', 'blue');
%     grid on;
%     xlabel('time step');
%     ylabel('angle (in degrees)');
%     title('angle between pur\_vel and evad\_center\_vel');


% ----------------------- plotting the angle between pursuer_velocity and pursuer-evader_center -----------------------
%     costheta = zeros(hp.number_interval,1);
%     for t = 1:hp.number_interval
%         costheta(t,1) = dot(pursuer_velocity(:,t),pursuer_evader_center_vector(:,t))/(norm(pursuer_velocity(:,t))*norm(pursuer_evader_center_vector(:,t)));
%     end
%     theta = acos(costheta)*180/pi;
%     subplot(2,2,4);plot(theta, '.-', 'color', 'blue');
%     grid on;
%     xlabel('time step');
%     ylabel('angle (in degrees)');
%     title('angle between pur\_vel and pur\_evad\_center\_vec');


% ----------------------- plotting the angle between pursuer-evader_center and evader_vector -----------------------
%     costheta = zeros(hp.number_interval+1,1);
%     for t = 1:hp.number_interval+1
%         v1 = evader_center_trajectory(:,t)-pursuer_trajectory(:,t);
%         v2 = evader_trajectory(3:4,t) - evader_trajectory(1:2,t);
%         costheta(t,1) = dot(v1, v2)/(norm(v1)*norm(v2));
%     end
%     theta = acos(costheta)*180/pi;
%     subplot(1,1,1);plot(theta, '.-', 'color', 'blue');
%     grid on;
%     xlabel('time step');
%     ylabel('angle (in degrees)');
%     title('angle between pur-evad\_center\_vec and evad\_sep_vec');
%     
    % savefilename_fig = strcat(root_dir, 'combined-feature-plot-fig/', int2str(index), '.fig');
    % savefilename_png = strcat(root_dir, 'combined-feature-plot-png/', int2str(index), '.png');
    % savefig(h1, savefilename_fig);
    % saveas(h1, savefilename_png)


% ----------------------- plotting the angle of the evader-vector measured wrt X-axis -----------------------
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


% ----------------------- checking whether the two triangles are similar -----------------------
%     pursuer_evader_vector = evader_trajectory - repmat(pursuer_trajectory,[hp.number_evader,1]);
%     evader_evader_vector = evader_trajectory(1:2,:) - evader_trajectory(3:4,:);
%     pursuer_evader_distance = [sqrt(sum(pursuer_evader_vector(1:2,:).^2,1));sqrt(sum(pursuer_evader_vector(3:4,:).^2,1))];
%     evader_evader_distance = sqrt(sum(evader_evader_vector.^2,1));
%     similarity_ratio = [pursuer_evader_distance(:,2:hp.number_interval+1)./pursuer_evader_distance(:,1:hp.number_interval);...
%         evader_evader_distance(:,2:hp.number_interval+1)./evader_evader_distance(:,1:hp.number_interval)];


% ----------------------- plotting the angle between pursuer-velocity and destination-evader_center vector -----------------------
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


% ----------------------- angle between the pursuer-evader_center and destination-evader_center -----------------------
    % theta = zeros(hp.number_interval,1);
    % for t = 1:hp.number_interval
    %     a = pursuer_trajectory(:,t) - evader_center_trajectory(:,t);
    %     b = hp.destination - evader_center_trajectory(:,t);
    %     theta(t,1) = acos(dot(a,b)/(norm(a)*norm(b)))*(180/pi);
    % end
    
    % h6 = figure;
    % plot(theta, 'o-', 'color', 'blue');hold on;
    % grid on;
    % xlabel('time step');
    % ylabel('angle (in degrees)');
    % title('angle between pursuer\_position and destination wrt evader\_center_trajectory');
    % hold off;
    % savefilename = strcat(strcat(root_dir,'feature_6/'),strcat(int2str(index),'.fig'));
    % savefig(h6, savefilename);

%     close all;
end