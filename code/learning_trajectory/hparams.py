# rnn-parameters
time_window_size = 5
cell_state_sizes = [32, 64]

# num_evader = 2
# num_pursuer = 1
num_outputs = 2

# fc-layer-parameters
num_fc_units = [32, 8, 2]

# training-parameters
num_epochs = 500
learning_rate = 1e-3
keep_prob_ = 0.8
max_gradient_norm = 10

input_feature_size = 8
epsilon = 0.05       # the destination circle radius

css = ''             # refers to cell state size
for i in range(len(cell_state_sizes)):
    css = css + str(cell_state_sizes[i]) + '_'

nfu = ''             # refers to fully connected units
for i in range(len(num_fc_units)):
    nfu = nfu + str(num_fc_units[i]) + '_'

project_dir = 'drive/My Drive/Academics/8thsem/btech-project/'

data_base_dir = project_dir+'data/learning_trajectory/'
train_data_file_path = data_base_dir+'dataset_train.csv'
test_data_file_path = data_base_dir+'dataset_train.csv'

model_id = 'tws_'+str(time_window_size)+'_css_'+css+'fc_'+nfu+'lr_'+str(learning_rate) \
        +'_es_'+str(num_epochs)+'/'

result_dir = project_dir+'results_plots/relevant_results/learned_trajectories/new_results/'+model_id

model_base_dir = result_dir+'model_save_dir/'
# model_save_dir = model_base_dir+model_id+'model_save_dir/'
model_save_filename = model_base_dir+'model'

plt_base_dir = result_dir+'plots/'
# plt_save_dir = plt_base_dir+

summary_base_dir = result_dir
summary_dir = summary_base_dir+'graph/'

train_bool = True
test_bool = True