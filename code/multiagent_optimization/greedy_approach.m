clear; clc;

% ---------------------hyper-parameters------------------------
hp.number_interval = 70;
hp.time_interval = 1.0;
hp.number_evader = 2;
hp.number_pursuer = 1;
hp.vemax_repulsion = 0.3;
hp.vemax_attraction = 0;
hp.vp = 0.2;
hp.K = 1.0;
hp.epsilon = 0.05;
hp.angle_resolution = 1;
hp.alpha = 0.6;      % weight factor

% hp.initial_pursuer_position = file.initial_pursuer_position;
hp.initial_pursuer_position = [-1;-1];

% hp.initial_evader_position = rand(2*hp.number_evader,1)*2-1;
% hp.initial_evader_position = file.initial_evader_position;
hp.initial_evader_position = [0.5;0;-0.5;0];

hp.destination = [2;2];
% ---------------------------hyper-parameters------------------------

pursuer_trajectory = zeros(2*hp.number_pursuer,hp.number_interval+1);
evader_trajectory = zeros(2*hp.number_evader,hp.number_interval+1);
pursuer_trajectory(:,1) = hp.initial_pursuer_position;
evader_trajectory(:,1) = hp.initial_evader_position;
distance_list = zeros(4,hp.number_interval+1);

for t = 1:hp.number_interval
    metric_list = zeros(360/hp.angle_resolution,1);
	for angle = 0:hp.angle_resolution:359.999
		pursuer_evader_vector = reshape(evader_trajectory(:,t),2,hp.number_evader) - repmat(pursuer_trajectory(:,t),[1,hp.number_evader]);
		pursuer_evader_distance = sqrt(sum(pursuer_evader_vector.^2,1));
		pursuer_temp_position = pursuer_trajectory(:,t) + [hp.vp*cos(angle);hp.vp*sin(angle)]*hp.time_interval;
		pursuer_velocity = [hp.vp*cos(angle);hp.vp*sin(angle)];
		costheta = (transpose(pursuer_velocity)*pursuer_evader_vector)./(pursuer_evader_distance*norm(pursuer_velocity));
		repulsion_term = reshape(0.5*hp.vemax_repulsion*repmat(exp(-hp.K*pursuer_evader_distance).*(1+costheta),[2,1]).*(pursuer_evader_vector./repmat(pursuer_evader_distance,[2,1])),2*hp.number_evader,1);
		evader_temp_position = evader_trajectory(:,t) + repulsion_term*hp.time_interval;
		evader_destination_vector = reshape(evader_temp_position,2,hp.number_evader) - repmat(hp.destination,[1,hp.number_evader]);
		evader_destination_total_distance = sum(sqrt(sum(evader_destination_vector.^2,1)));
        inter_evader_distance = norm(evader_temp_position(1:2,1) - evader_temp_position(3:4,1));
		metric_list(angle/hp.angle_resolution+1,1) = hp.alpha*evader_destination_total_distance + (1-hp.alpha)*inter_evader_distance;
	end
	[~,loc] = min(metric_list);
    optimal_angle = (loc-1)*hp.angle_resolution;
	pursuer_trajectory(:,t+1) = pursuer_trajectory(:,t) + [hp.vp*cos(optimal_angle);hp.vp*sin(optimal_angle)]*hp.time_interval;
	pursuer_evader_vector = reshape(evader_trajectory(:,t),2,hp.number_evader) - repmat(pursuer_trajectory(:,t),[1,hp.number_evader]);
	pursuer_evader_distance = sqrt(sum(pursuer_evader_vector.^2,1));
	pursuer_velocity = (pursuer_trajectory(:,t+1) - pursuer_trajectory(:,t))/hp.time_interval;
	costheta = (transpose(pursuer_velocity)*pursuer_evader_vector)./(pursuer_evader_distance*norm(pursuer_velocity));
	repulsion_term = reshape(0.5*hp.vemax_repulsion*repmat(exp(-hp.K*pursuer_evader_distance).*(1+costheta),[2,1]).*(pursuer_evader_vector./repmat(pursuer_evader_distance,[2,1])),2*hp.number_evader,1);
	evader_trajectory(:,t+1) = evader_trajectory(:,t) + repulsion_term*hp.time_interval;
    evader_dest_distance = sqrt(sum((reshape(evader_trajectory(:,t),2,hp.number_evader) - repmat(hp.destination,[1,hp.number_evader])).^2,1));
    inter_evader_distance = norm(evader_trajectory(1:2,t) - evader_trajectory(3:4,t));
    distance_list(:,t) = [transpose(evader_dest_distance);inter_evader_distance;hp.alpha*sum(evader_dest_distance)+(1-hp.alpha)*inter_evader_distance];
end
evader_dest_distance = sqrt(sum((reshape(evader_trajectory(:,hp.number_interval+1),2,hp.number_evader) - repmat(hp.destination,[1,hp.number_evader])).^2,1));
inter_evader_distance = norm(evader_trajectory(1:2,hp.number_interval+1) - evader_trajectory(3:4,hp.number_interval+1));
distance_list(:,hp.number_interval+1) = [transpose(evader_dest_distance);inter_evader_distance;hp.alpha*sum(evader_dest_distance)+(1-hp.alpha)*inter_evader_distance];

figure;
plot(pursuer_trajectory(1,:), pursuer_trajectory(2,:), '.-', 'color', 'blue');hold on;
% for i=1:hp.number_evader
%     plot(evader_trajectory(2*i-1,1),evader_trajectory(2*i,1), '.-', 'color', 'green');hold on;
%     plot(evader_trajectory(2*i-1,2:hp.number_interval),evader_trajectory(2*i,2:hp.number_interval),'.-','color','yellow');hold on;
%     plot(evader_trajectory(2*i-1,hp.number_interval+1),evader_trajectory(2*i,hp.number_interval+1), '.-', 'color', 'red');hold on;
% end

for t = 1:hp.number_interval
   plot([evader_trajectory(1,t),evader_trajectory(3,t)],[evader_trajectory(2,t),evader_trajectory(4,t)],'r');
end

draw_circle(hp.destination(1,1), hp.destination(2,1), hp.epsilon);
grid on;
xlabel('X');
ylabel('Y');
title('shepherding-greedy-approach');
hold off;

figure;
plot(distance_list(1,:),'.-','color','red');hold on;
plot(distance_list(2,:),'.-','color','green');hold on;
plot(distance_list(3,:),'.-','color','blue');hold on;
plot(distance_list(4,:),'.-','color','black');hold on;
grid on;
ylabel('distance-metric (J)');
xlabel('time');
legend('E1-dest-distance','E2-dest-distance','E1-E2-distance','overall-metric');
title('distance-metric plot with time');
hold off;