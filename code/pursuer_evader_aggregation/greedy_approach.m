clear; clc;

% ---------------------hyper-parameters------------------------
hp.number_interval = 28;
hp.time_interval = 1.0;
hp.number_evader = 2;
hp.number_pursuer = 1;
hp.vemax_repulsion = 0.4;
hp.vemax_attraction = 0;
hp.vp = 0.1;
hp.K = 1.0;
hp.epsilon = 0.05;

hp.var = 2*hp.number_pursuer;
hp.N = hp.var*hp.number_interval;

% file = load('data_file.mat');

% hp.initial_pursuer_position = file.initial_pursuer_position;
hp.initial_pursuer_position = [-1;-1];

% hp.initial_evader_position = rand(2*hp.number_evader,1)*2-1;
% hp.initial_evader_position = file.initial_evader_position;
hp.initial_evader_position = [0;0;1;1];

hp.destination = [2;2];
% ---------------------------hyper-parameters------------------------

pursuer_trajectory = zeros(2*hp.number_pursuer,hp.number_interval+1);
evader_trajectory = zeros(2*hp.number_evader,hp.number_interval+1);
pursuer_trajectory(:,1) = hp.initial_pursuer_position;
evader_trajectory(:,1) = hp.initial_evader_position;
distance_list = zeros(360,1);

% while sum(sqrt(sum((reshape(evader_trajectory(:,t),2,hp.number_evader) - repmat(hp.destination,[1,hp.number_evader])).^2,1)) > hp.epsilon) ~= 0
for t = 1:hp.number_interval
	for angle = 1:1:360
		pursuer_evader_vector = reshape(evader_trajectory(:,t),2,hp.number_evader) - repmat(pursuer_trajectory(:,t),[1,hp.number_evader]);
		pursuer_evader_distance = sqrt(sum(pursuer_evader_vector.^2,1));
		pursuer_temp_position = pursuer_trajectory(:,t) + [hp.vp*cos(angle);hp.vp*sin(angle)]*hp.time_interval;
		pursuer_velocity = (pursuer_temp_position - pursuer_trajectory(:,t))/hp.time_interval;
		costheta = (transpose(pursuer_velocity)*pursuer_evader_vector)./(pursuer_evader_distance*norm(pursuer_velocity));
		repulsion_term = reshape(0.5*hp.vemax_repulsion*repmat(exp(-hp.K*pursuer_evader_distance).*(1+costheta),[2,1]).*(pursuer_evader_vector./repmat(pursuer_evader_distance,[2,1])),2*hp.number_evader,1);
		evader_temp_position = evader_trajectory(:,t) + repulsion_term*hp.time_interval;
		evader_destination_vector = reshape(evader_temp_position,2,hp.number_evader) - repmat(hp.destination,[1,hp.number_evader]);
		evader_destination_total_distance = sum(sqrt(sum(evader_destination_vector.^2,1)));
		distance_list(angle,1) = evader_destination_total_distance;
	end
	[~,loc] = min(distance_list);
	pursuer_trajectory(:,t+1) = pursuer_trajectory(:,t) + [hp.vp*cos(loc);hp.vp*sin(loc)]*hp.time_interval;
	pursuer_evader_vector = reshape(evader_trajectory(:,t),2,hp.number_evader) - repmat(pursuer_trajectory(:,t),[1,hp.number_evader]);
	pursuer_evader_distance = sqrt(sum(pursuer_evader_vector.^2,1));
	pursuer_velocity = (pursuer_trajectory(:,t+1) - pursuer_trajectory(:,t))/hp.time_interval;
	costheta = (transpose(pursuer_velocity)*pursuer_evader_vector)./(pursuer_evader_distance*norm(pursuer_velocity));
	repulsion_term = reshape(0.5*hp.vemax_repulsion*repmat(exp(-hp.K*pursuer_evader_distance).*(1+costheta),[2,1]).*(pursuer_evader_vector./repmat(pursuer_evader_distance,[2,1])),2*hp.number_evader,1);
	evader_trajectory(:,t+1) = evader_trajectory(:,t) + repulsion_term*hp.time_interval;
end


figure;
plot(pursuer_trajectory(1,:), pursuer_trajectory(2,:), 'o-', 'color', 'blue');hold on;
for i=1:hp.number_evader
    plot(evader_trajectory(2*i-1,1),evader_trajectory(2*i,1), 'o-', 'color', 'green');hold on;
    plot(evader_trajectory(2*i-1,2:hp.number_interval),evader_trajectory(2*i,2:hp.number_interval),'o-','color','yellow');hold on;
    plot(evader_trajectory(2*i-1,hp.number_interval+1),evader_trajectory(2*i,hp.number_interval+1), 'o-', 'color', 'red');hold on;
end
for t = 1:hp.number_interval
   plot([evader_trajectory(1,t),evader_trajectory(3,t)],[evader_trajectory(2,t),evader_trajectory(4,t)],'r');
end

draw_circle(hp.destination(1,1), hp.destination(2,1), hp.epsilon);
grid on;
xlabel('X');
ylabel('Y');
title('shepherding-optimization-result');
hold off;