clc;clear;

% feature_list = ["pursuer_position_x"; "pursuer_position_y"; "evader1_position_x"; "evader1_position_y"; ...
%     "evader2_position_x"; "evader2_position_y"; "destination_x"; "destination_y"; "pursuer_velocity_x"; ...
%     "pursuer_velocity_y"; "evader1_velocity_x"; "evader1_velocity_y"; "evader2_velocity_x"; ...
%     "evader2_velocity_y"];

feature_list = ["pursuer_position_x"; "pursuer_position_y"; "evader1_position_x"; "evader1_position_y"; ...
    "evader2_position_x"; "evader2_position_y"; "destination_x"; "destination_y";"vemax_repulsion";];

num_input_features = size(feature_list,1);
num_outputs = 2;
addpath('../pursuer_evader_aggregation/');
root_dir = '../../results-plots/';
folder_dirs = ["15-03-19/";"19-03-19/";"05-04-19/"];
rows_final_matrix = 0;

for folder_index = 1:size(folder_dirs,1)
    file_dir = strcat(root_dir, folder_dirs(folder_index,1), 'hyperparameters/');
    for file_index = 1:size(dir(strcat(file_dir,'*.mat')),1)
        filename = strcat(file_dir, int2str(file_index), '.mat');
        load(filename);
        matrix = zeros(num_input_features+num_outputs, hp.number_interval+1+1);
        
        pursuer_trajectory = horzcat(hp.initial_pursuer_position,reshape(hp.opt_x,hp.var,hp.number_interval));
        evader_trajectory = compute_evader_position(pursuer_trajectory,hp.number_evader,hp.initial_evader_position,...
            hp.number_interval,hp.time_interval,hp.vemax_repulsion,hp.vemax_attraction,hp.K);
        
        pursuer_velocity = pursuer_trajectory(:,2:end) - pursuer_trajectory(:,1:end-1);
        evader_velocity = evader_trajectory(:,2:end) - evader_trajectory(:,1:end-1);    
        
        matrix(1:2,1:end-1) = pursuer_trajectory;
        matrix(3:6,1:end-1) = evader_trajectory;
        matrix(7:8,1:end-1) = repmat(hp.destination,[1,hp.number_interval+1]);
        matrix(9,1:end-1) = hp.vemax_repulsion;
%         matrix(9:10,1) = 0;
%         matrix(9:10,2:end-1) = pursuer_velocity;
%         matrix(11:14,1) = 0;
%         matrix(11:14,2:end-1) = evader_velocity;
        matrix(end-1:end,1:end-2) = pursuer_trajectory(:,2:end);
        matrix(end-1:end,end-1) = pursuer_trajectory(:,end);
%         matrix(17:20,1:end-2) = evader_trajectory(:,2:end);
%         matrix(17:20,end-1) = evader_trajectory(:,end);
        matrix(:,end) = 0;
        matrix = matrix';
        final_matrix(rows_final_matrix+1:rows_final_matrix+size(matrix,1),:) = matrix;
        rows_final_matrix = size(final_matrix,1);
    end
end
csvwrite('data/dataset_train.csv', final_matrix);
csvwrite('data/dataset_test.csv', final_matrix);

% test_data = final_matrix(1605:1635,:);
% csvwrite('dataset_test.csv', test_data);