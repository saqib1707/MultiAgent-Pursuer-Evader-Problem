import numpy as np 
import tensorflow as tf
import matplotlib.pyplot as plt 
import csv
import time
import os
import pdb
import shutil
import hyperparameters as hp


def normalize_data(data):
    global data_mean, data_std
    data_mean = np.mean(data[np.where(data.any(axis=1) != 0)[0]], axis=0)
    data_std = np.std(data[np.where(data.any(axis=1) != 0)[0]], axis=0)
    data[np.where(data.any(axis=1) != 0)[0]] = (data[np.where(data.any(axis=1) != 0)[0]] - data_mean)/data_std

    # saving the normalized data in csv format for visualization/analysis
    # np.savetxt("normalized_dataset.csv", data, delimiter=",", fmt='%1.5f')
    return data


def get_data(train_data_file_path, test_data_file_path):
    train_data = []
    test_data = []
    with open(train_data_file_path) as csvfile:
        readCSV = csv.reader(csvfile, delimiter=',')
        for row in readCSV:
            train_data.append(row)

    with open(test_data_file_path) as csvfile:
        readCSV = csv.reader(csvfile, delimiter=',')
        for row in readCSV:
            test_data.append(row)
    train_data = np.array(train_data, dtype=np.float32)
    test_data = np.array(test_data, dtype=np.float32)
    # data = normalize_data(data)

    train_x = train_data[:,0:hp.feature_size]
    train_y = train_data[:,hp.feature_size:hp.feature_size+hp.num_outputs]
    test_x = test_data[:,0:hp.feature_size]
    test_y = test_data[:,hp.feature_size:hp.feature_size+hp.num_outputs]

    return train_x, train_y, test_x, test_y


def get_next_full_batch(data_x, data_y, step, zero_row_indices):
    if step == 0:
        samples_range = range(0, zero_row_indices[step])
    else:
        samples_range = range(zero_row_indices[step-1]+1, zero_row_indices[step])

    batch_size = len(samples_range)-hp.window_size+1
    batch_x = np.zeros((batch_size, hp.window_size, hp.feature_size))
    batch_y = np.zeros((batch_size, hp.num_outputs))

    for i in range(batch_size):
        batch_x[i] = data_x[samples_range[0]+i:samples_range[0]+i+hp.window_size]
        batch_y[i] = data_y[samples_range[0]+i+hp.window_size]

    return batch_x, batch_y


def get_next_batch(data_x, data_y, index):
    batch_x = np.zeros((hp.batch_size, hp.window_size, hp.feature_size))
    batch_y = np.zeros((hp.batch_size, hp.num_outputs))
    
    for i in range(hp.batch_size):
        for j in range(hp.window_size):
            batch_x[i,j] = data_x[index*hp.batch_size+i+j]
        batch_y[i] = data_y[index*hp.batch_size+i+hp.window_size-1]

    return batch_x, batch_y


def compute_evader_position(pursuer_next_position, pursuer_current_position, evader_current_position, ti, vemax_repulsion, vemax_attraction, K):
    pursuer_evader_vector = np.transpose(np.reshape(evader_current_position, (2,2))) - pursuer_current_position  # (2,2)
    pursuer_velocity = (pursuer_next_position - pursuer_current_position)/ti    # (2,1)
    pursuer_evader_distance = np.reshape(np.linalg.norm(pursuer_evader_vector, axis=0), (2,1))     # (2,1)
    costheta = np.matmul(np.transpose(pursuer_evader_vector), pursuer_velocity)/(pursuer_evader_distance*np.linalg.norm(pursuer_velocity)) # (2,1)
    # evader_position_sum = sum(reshape(evader_position(:,t),2,Ne),2);
    # centroid_nearest_neighbour = (repmat(evader_position_sum,[1,Ne]) - reshape(evader_position(:,t),2,Ne))/(Ne-1);
    # centroid_evader_vector = centroid_nearest_neighbour - reshape(evader_position(:,t),2,Ne);
    # centroid_evader_distance = sqrt(sum(centroid_evader_vector.^2,1));
    repulsion_term = np.reshape(np.transpose(0.5*vemax_repulsion*np.transpose(np.exp(-K*pursuer_evader_distance)*(1+costheta))*
        (pursuer_evader_vector/np.transpose(pursuer_evader_distance))), (4,1));
    # attraction_term = reshape(vemax_attraction*repmat(exp(-K*pursuer_evader_distance),[2,1]).*(centroid_evader_vector./
    # repmat(centroid_evader_distance,[2,1])),2*Ne,1);
    evader_next_position = evader_current_position + repulsion_term*ti #+ attraction_term*ti;
    return evader_next_position


def build_lstm_network(lstm_state_size, rnn_inputs, keep_prob_):
    lstms = [tf.nn.rnn_cell.LSTMCell(num_units=size) for size in lstm_state_size]
    # dropout_lstms = [tf.nn.rnn_cell.DropoutWrapper(cell=lstm, output_keep_prob=keep_prob_) for lstm in lstms]
    # multi_rnn_cell = tf.nn.rnn_cell.MultiRNNCell(dropout_lstms)

    # lstm_outputs.shape = [batch_size, window_size, lstm_state_size[1]]
    lstm_outputs, final_state = tf.nn.dynamic_rnn(cell=lstms[0], inputs=rnn_inputs, dtype=tf.float32)
    return lstm_outputs, final_state

def create_fc_layer(input_data, num_input_units, num_output_units, variable_scope_name, activation='linear'):
    init = tf.contrib.layers.xavier_initializer(uniform=True, dtype=tf.float32)
    with tf.variable_scope(variable_scope_name, reuse=tf.AUTO_REUSE):
        weight = tf.get_variable(name='w', shape=[num_input_units, num_output_units], initializer=init, trainable=True)
        bias = tf.get_variable(name='b', shape=[num_output_units], initializer=init, trainable=True)
    
    fc_layer_output = tf.add(tf.matmul(tf.reshape(input_data, [-1, num_input_units]), weight), bias)
    
    if activation == 'relu':
        fc_layer_output = tf.nn.relu(fc_layer_output)
    elif activation == 'tanh':
        fc_layer_output = tf.nn.tanh(fc_layer_output)
    elif activation == 'sigmoid':
        fc_layer_output = tf.nn.sigmoid(fc_layer_output)
    else:
        pass

    return fc_layer_output

def core_model(lstm_state_size, inputs, keep_prob_):
    lstm_outputs, final_state = build_lstm_network(lstm_state_size, inputs, keep_prob_)
    lstm_outputs = lstm_outputs[:,-1,:]        # taking the final state output (many-to-one)

    fc_layer_output = create_fc_layer(lstm_outputs, lstm_state_size[-1], hp.num_fc_layer_units, 'first_fc_layer', 'relu')  # fc_layer_output.shape = [batch_size, num_fc_layer_units]
    logits = create_fc_layer(fc_layer_output, hp.num_fc_layer_units, hp.num_outputs, 'final_fc_layer')  # logits.shape = [batch_size, num_outputs]

    # logits = create_fc_layer(lstm_outputs, lstm_state_size[-1], hp.num_outputs, 'final_fc_layer')
    return logits


def gradient_clip(gradients, max_gradient_norm):
    clipped_gradients, global_norm = tf.clip_by_global_norm(t_list=gradients, clip_norm=max_gradient_norm)  # (t_list*clip_norm)/max(clip_norm, global_norm)
    gradient_norm_summary = [tf.summary.scalar("grad_norm", global_norm)]
    gradient_norm_summary.append(tf.summary.scalar("clipped_gradient", tf.global_norm(clipped_gradients)))
    return clipped_gradients, global_norm, gradient_norm_summary


def train_network(train_x, train_y, test_x, test_y):
    start_time = time.time()
    network_input = tf.placeholder(dtype=tf.float32, shape=[None, hp.window_size, hp.feature_size], name='input_placeholder')
    network_output = tf.placeholder(dtype=tf.float32, shape=[None, hp.num_outputs], name='label_placeholder')   # ground-truth
    
    num_training_samples = train_x.shape[0]
    zero_row_indices = np.where(train_x.any(axis=1) == 0)[0]
    len_zero_row_indices = zero_row_indices.shape[0]

    logits = core_model(hp.lstm_state_size, network_input, hp.keep_prob_)    # logits.shape = [batch_size, num_outputs]
    total_loss = tf.losses.mean_squared_error(labels=network_output, predictions=tf.reshape(logits, [-1, hp.num_outputs]))
    tf.summary.scalar('mean_squared_loss', total_loss)

    # optimizer.minimize(total_loss) = optimizer.compute_gradients() -> optimizer.apply_gradients()
    optimizer = tf.train.AdamOptimizer(hp.learning_rate)

    with tf.name_scope("compute_gradients"):
        grads_and_params = optimizer.compute_gradients(loss=total_loss)
        grads, params = zip(*grads_and_params)
        # for var in params:
        #     tf.summary.histogram(var.name, var)
        clipped_grads, global_norm, grad_norm_summary = gradient_clip(gradients=grads, max_gradient_norm=hp.max_gradient_norm)
        grad_and_vars = zip(clipped_grads, params)

    # global_step refers to the number of batches seen by the graph. Every time a batch is provided, the weights are updated in the direction 
    # that minimizes the loss. global_step just keeps track of the number of batches seen so far
    global_step = tf.train.get_or_create_global_step()
    apply_gradient_op = optimizer.apply_gradients(grads_and_vars=grad_and_vars, global_step=global_step)

    session_config = tf.ConfigProto(allow_soft_placement=True, log_device_placement=False)
    session_config.gpu_options.allow_growth = True
    sess = tf.Session(config=session_config)
    train_writer = tf.summary.FileWriter(hp.log_dir ,sess.graph)

    # This initializer is designed to keep the scale of the gradients roughly the same in all layers
    initializer = tf.contrib.layers.xavier_initializer(uniform=True, dtype=tf.float32)

    with tf.variable_scope(tf.get_variable_scope(), initializer=initializer): 
        sess.run(tf.global_variables_initializer())
        merged = tf.summary.merge_all()

        print("Training Started")
        for epoch in range(hp.num_training_epochs):
            step = 0
            is_epoch_complete = False
            while is_epoch_complete == False:
                [batch_x, batch_y] = get_next_full_batch(train_x, train_y, step, zero_row_indices)
                _, loss_val, summary, iteration_count = sess.run([apply_gradient_op, total_loss, merged, global_step], feed_dict={network_input:batch_x, network_output:batch_y})
                train_writer.add_summary(summary, iteration_count)
                if iteration_count%10 == 0:
                    print("Epoch :", epoch, "iteration :", iteration_count)
                step = step + 1
                is_epoch_complete = True if (step == len_zero_row_indices) else False

        print("Training Ended")
        
        # Testing the network
        test_data_x = np.reshape(np.copy(test_x[0:hp.window_size]), (1, hp.window_size, hp.feature_size))

        pursuer_trajectory = []
        evader_trajectory = []

        for i in range(hp.window_size):
            pursuer_trajectory.append(np.reshape(np.copy(test_x[i,0:2]), (2,1)))
            evader_trajectory.append(np.reshape(np.copy(test_x[i,2:6]), (4,1)))

        number_interval = 30
        for index in range(number_interval):
            pursuer_next_position = np.reshape(sess.run([logits], feed_dict={network_input:test_data_x}), (2,1))
            evader_next_position = compute_evader_position(pursuer_next_position, pursuer_trajectory[index+hp.window_size-1], 
                evader_trajectory[index+hp.window_size-1], ti=1.0, vemax_repulsion=0.4, vemax_attraction=0, K=1.0)

            test_data_x[0, 0:hp.window_size-1, :] = np.copy(test_data_x[0, 1:hp.window_size, :])
            test_data_x[0, hp.window_size-1, 0:2] = np.reshape(np.copy(pursuer_next_position), (2))
            test_data_x[0, hp.window_size-1, 2:6] = np.reshape(np.copy(evader_next_position), (4))

            pursuer_trajectory.append(pursuer_next_position)
            evader_trajectory.append(evader_next_position)
            # index += 1
            # if np.linalg.norm(np.reshape(evader_next_position[0:2,:],(2,1)) - np.array([[feature_dict['destination_x']],[feature_dict['destination_y']]])) < 0.1 and \
            #     np.linalg.norm(np.reshape(evader_next_position[2:4,:],(2,1)) - np.array([[feature_dict['destination_x']],[feature_dict['destination_y']]])) < 0.1:
            #     metric = False

        pursuer_trajectory = np.transpose(np.array(pursuer_trajectory)[:,:,0])  # (2,number_interval)
        evader_trajectory = np.transpose(np.array(evader_trajectory)[:,:,0])    # (4,number_interval)

        # pursuer_trajectory = pursuer_trajectory*np.reshape(data_std[0:2], (2,1))+np.reshape(data_mean[0:2], (2,1))
        # evader_trajectory = evader_trajectory*np.reshape(data_std[2:6], (4,1))+np.reshape(data_mean[2:6], (4,1))

    end_time = time.time()
    print("Training time : ", end_time - start_time)
    return pursuer_trajectory, evader_trajectory


def check_compute_evader_function(data_file_path):
    data = get_data(data_file_path)
    initial_index = 257
    end_index = 287
    pursuer_trajectory = np.transpose(data[initial_index-1:end_index+1,0:2])
    initial_evader_position = np.transpose(data[initial_index-1,2:6])
    evader_trajectory = []
    evader_trajectory.append(np.reshape(initial_evader_position,(4,1)))
    for index in range(end_index - initial_index):
        evader_trajectory.append(compute_evader_position(np.reshape(pursuer_trajectory[:,index+1],(2,1)), np.reshape(pursuer_trajectory[:,index], (2,1)), 
            evader_trajectory[index], 1.0, data[initial_index-1,10], 0, 1.0))

    evader_trajectory = np.array(evader_trajectory)
    evader_trajectory = evader_trajectory[:,:,0]
    plt.plot(pursuer_trajectory[0,:-1],pursuer_trajectory[1,:-1])
    plt.plot(evader_trajectory[:,0],evader_trajectory[:,1])
    plt.plot(evader_trajectory[:,2],evader_trajectory[:,3])
    plt.grid(True)
    plt.show()


def draw_circle(x_center, y_center, radius, angle_resolution=0.1):
    angles = np.linspace(0, 2*np.pi, 360/angle_resolution)
    xp = radius*np.cos(angles)
    yp = radius*np.sin(angles)
    plt.plot(x_center+xp, y_center+yp, 'black')


if __name__ == '__main__':
    train_data_file_path = 'dataset_train.csv'
    test_data_file_path = 'dataset_test.csv'
    train_x, train_y, test_x, test_y = get_data(train_data_file_path, test_data_file_path)
    if os.path.isdir(hp.log_dir) == True:
        shutil.rmtree(hp.log_dir)
        os.makedirs(hp.log_dir)
    else:
        os.makedirs(hp.log_dir)
    pursuer_trajectory, evader_trajectory = train_network(train_x, train_y, test_x, test_y)
    # print("Pursuer Trajectory :", pursuer_trajectory)
    # print("Evader Trajectory :", evader_trajectory)

    plt.plot(pursuer_trajectory[0,:], pursuer_trajectory[1,:], 'b', lw=1, marker='.')
    plt.plot(evader_trajectory[0,:], evader_trajectory[1,:], 'y', lw=1, marker='.')
    plt.plot(evader_trajectory[2,:], evader_trajectory[3,:], 'y', lw=1, marker='.')
    
    destination = test_x[0,6:8]
    draw_circle(destination[0], destination[1], hp.epsilon)

    plt.title('Shepherding-prediction-result')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.grid(True)
    plt.show()