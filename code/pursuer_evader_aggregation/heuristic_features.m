clear;clc;
load('../../results-plots/15-03-19/hyperparameters/8.mat');
pursuer_trajectory = horzcat(hp.initial_pursuer_position,reshape(hp.opt_x,2,hp.number_interval));
evader_trajectory = compute_evader_position(pursuer_trajectory,hp.number_evader,hp.initial_evader_position,hp.number_interval,hp.time_interval,hp.vemax_repulsion,hp.vemax_attraction,hp.K);
evader_center_trajectory = (evader_trajectory(1:2,:)+evader_trajectory(3:4,:))/2;
figure;
plot(pursuer_trajectory(1,:), pursuer_trajectory(2,:), 'o-', 'color', 'blue');hold on;
for i=1:hp.number_evader
    plot(evader_trajectory(2*i-1,1),evader_trajectory(2*i,1), 'o-', 'color', 'green');hold on;
    plot(evader_trajectory(2*i-1,2:hp.number_interval),evader_trajectory(2*i,2:hp.number_interval),'o-','color','yellow');hold on;
    plot(evader_trajectory(2*i-1,hp.number_interval+1),evader_trajectory(2*i,hp.number_interval+1), 'o-', 'color', 'red');hold on;
end
for t = 1:hp.number_interval
    plot([evader_trajectory(1,t),evader_trajectory(3,t)],[evader_trajectory(2,t),evader_trajectory(4,t)],'r');
    plot([evader_center_trajectory(1,t),pursuer_trajectory(1,t)],[evader_center_trajectory(2,t),pursuer_trajectory(2,t)],'black');
    plot([evader_center_trajectory(1,t),hp.destination(1,1)],[evader_center_trajectory(2,t),hp.destination(2,1)],'red');
end
draw_circle(hp.destination(1,1), hp.destination(2,1), hp.epsilon);
grid on;
xlabel('X');
ylabel('Y');
title('shepherding-optimization-result');
hold off;

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

evader_separation = sqrt(sum((evader_trajectory(1:2,:) - evader_trajectory(3:4,:)).^2,1));
figure;
plot(evader_separation, 'o-', 'color', 'blue');hold on;
grid on;
xlabel('time step');
ylabel('evader separation');
title('separation between evaders');
hold off;
