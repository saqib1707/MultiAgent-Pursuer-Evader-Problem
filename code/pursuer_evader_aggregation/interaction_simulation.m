clear; clc;

% tuning hyper-parameters
number_interval = 5;
time_interval = 1.0;
number_evader = 5;
vemax_repulsion = 1.0;
vemax_attraction = 1.0;
vpmax = 0.5;
K = 1.0;

num_agents = number_evader+1;
initial_point = rand(num_agents*2,1)*2-1;
initial_point(num_agents*2-1:num_agents*2,1) = [0;0];

% m = (initial_point(2,1) - initial_point(4,1))/(initial_point(1,1) - initial_point(3,1));
m = 1/sqrt(3);
delx = vpmax*time_interval/sqrt(1+m*m);
dely = vpmax*time_interval*m/sqrt(1+m*m);
pursuer_position = zeros(2,number_interval+1);
evader_position = zeros(2*number_evader,number_interval+1);
pursuer_position(:,1) = [initial_point(2*number_evader+1,1);initial_point(2*number_evader+2,1)];
evader_position(:,1) = initial_point(1:2*number_evader,1);

for i=1:number_interval
    pursuer_position(:,i+1) = pursuer_position(:,i) + [delx;dely];
    pursuer_evader_vector = reshape(evader_position(:,i),2,number_evader) - repmat(pursuer_position(:,i),[1,number_evader]);
    pursuer_velocity = pursuer_position(:,i+1) - pursuer_position(:,i);
    pursuer_evader_distance = sqrt(sum(pursuer_evader_vector.^2,1));
    costheta = (transpose(pursuer_velocity)*pursuer_evader_vector)./(pursuer_evader_distance*norm(pursuer_velocity));
    evader_position_sum = sum(reshape(evader_position(:,i),2,number_evader),2);
    centroid_nearest_neighbour = (repmat(evader_position_sum,[1,number_evader]) - reshape(evader_position(:,i),2,number_evader))/(number_evader-1);
    centroid_evader_vector = centroid_nearest_neighbour - reshape(evader_position(:,i),2,number_evader);
    centroid_evader_distance = sqrt(sum(centroid_evader_vector.^2,1));
    repulsion_term = reshape(0.5*vemax_repulsion*repmat(exp(-K*pursuer_evader_distance).*(1+costheta),[2,1]).*(pursuer_evader_vector./repmat(pursuer_evader_distance,[2,1])), 2*number_evader, 1);
    attraction_term = reshape(vemax_attraction*repmat(exp(-K*pursuer_evader_distance),[2,1]).*(centroid_evader_vector./repmat(centroid_evader_distance,[2,1])), 2*number_evader, 1);
    evader_position(:,i+1) = evader_position(:,i) + repulsion_term + attraction_term;
end
figure;
plot(pursuer_position(1,:),pursuer_position(2,:), 'o-', 'color', 'blue');hold on;
for i=1:number_evader
    plot(evader_position(2*i-1,1),evader_position(2*i,1), 'o-', 'color', 'green');hold on;
    plot(evader_position(2*i-1,2:number_interval),evader_position(2*i,2:number_interval), 'o-', 'color', 'yellow');hold on;
    plot(evader_position(2*i-1,number_interval+1),evader_position(2*i,number_interval+1), 'o-', 'color', 'red');hold on;
end
grid on;
xlabel('X');
ylabel('Y');
title('pursuer-evader-interaction');
hold off;