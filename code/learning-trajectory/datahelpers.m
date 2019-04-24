clc;clear;
% order of the features : [pursuer_position_x, pursuer_position_y,
% [evader_position_x, evader_position_y]*number_evaders, destination_x,
% destination_y, vpmax, vpmin, vemax_repulsion, epsilon, EOS (0)]
addpath('../pursuer_evader_aggregation/');
root_dir = '../../results-plots/';
folder_dirs = ["15-03-19/";"19-03-19/";"05-04-19/"];
rows_final_matrix = 0;

for folder_index = 1:size(folder_dirs,1)
    file_dir = strcat(strcat(root_dir, folder_dirs(folder_index,1)), 'hyperparameters/');
    for file_index = 1:size(dir(strcat(file_dir,'*.mat')),1)
        filename = strcat(file_dir,strcat(int2str(file_index),'.mat'));
        load(filename);
        num_input_features = hp.number_pursuer*2 + hp.number_evader*2 + 2;
        matrix = zeros(num_input_features+2, hp.number_interval+1+1);
        pursuer_position = horzcat(hp.initial_pursuer_position,reshape(hp.opt_x,hp.var,hp.number_interval));
        evader_position = compute_evader_position(pursuer_position,hp.number_evader,hp.initial_evader_position,...
            hp.number_interval,hp.time_interval,hp.vemax_repulsion,hp.vemax_attraction,hp.K);

        matrix(1:2,1:end-1) = pursuer_position;
        matrix(3:6,1:end-1) = evader_position;
        matrix(7:8,1:end-1) = repmat(hp.destination,[1,hp.number_interval+1]);
        % matrix(9,1:end-1) = hp.vpmax;
        % matrix(10,1:end-1) = hp.vpmin;
        % matrix(11,1:end-1) = hp.vemax_repulsion;
        % matrix(12,1:end-1) = hp.epsilon;
        matrix(end-1:end,1:end-2) = pursuer_position(:,2:end);
        matrix(end-1:end,end-1) = pursuer_position(:,end);
        matrix(:,end) = 0;
        matrix = matrix';
        final_matrix(rows_final_matrix+1:rows_final_matrix+size(matrix,1),:) = matrix;
        rows_final_matrix = size(final_matrix,1);
    end
end
csvwrite('dataset_train.csv', final_matrix);

test_data = final_matrix(289:319,:);
csvwrite('dataset_test.csv', test_data);