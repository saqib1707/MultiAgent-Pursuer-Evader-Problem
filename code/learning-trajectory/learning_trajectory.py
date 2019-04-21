import numpy as np 
import tensorflow as tf
import matplotlib.pyplot as plt 
import csv
import time
import os
import hyperparameters as hp


def get_data(data_file_path):
    data = []
    with open(data_file_path) as csvfile:
        readCSV = csv.reader(csvfile, delimiter=',')
        for row in readCSV:
            data.append(row)
    data = np.array(data, dtype=np.float32)

    train_x = data[0:hp.num_training_samples,0:hp.feature_size]
    train_y = data[0:hp.num_training_samples,hp.feature_size:hp.feature_size+hp.num_outputs]
    test_x = data[hp.num_training_samples:,0:hp.feature_size]
    test_y = data[hp.num_training_samples:,hp.feature_size:hp.feature_size+hp.num_outputs]

    return train_x, train_y, test_x, test_y


def get_next_full_batch(data_x, data_y, step, zero_row_indices):
    is_epoch_complete = False
    len_zero_row_indices = zero_row_indices.shape[0]
    
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

    step = step + 1
    if (step == len_zero_row_indices):
        is_epoch_complete = True
    return batch_x, batch_y, step, is_epoch_complete


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
    dropout_lstms = [tf.nn.rnn_cell.DropoutWrapper(cell=lstm, output_keep_prob=keep_prob_) for lstm in lstms]
    multi_rnn_cell = tf.nn.rnn_cell.MultiRNNCell(dropout_lstms)

    # lstm_outputs.shape = [batch_size, window_size, lstm_state_size[1]]
    lstm_outputs, final_state = tf.nn.dynamic_rnn(cell=multi_rnn_cell, inputs=rnn_inputs, dtype=tf.float32)
    return lstm_outputs, final_state


def core_model(lstm_state_size, inputs, keep_prob_):
    lstm_outputs, final_state = build_lstm_network(lstm_state_size, inputs, keep_prob_)
    lstm_outputs = lstm_outputs[:,-1,:]        # taking the final state output (many-to-one)
    
    with tf.variable_scope('first_fc_layer', reuse=tf.AUTO_REUSE):
        weight = tf.get_variable(name='w', shape=[lstm_state_size[1], hp.num_fc_layer_units])
        bias = tf.get_variable(name='b', shape=[hp.num_fc_layer_units], initializer=tf.constant_initializer(0.0))
    fc_layer_output = tf.nn.relu(tf.add(tf.matmul(tf.reshape(lstm_outputs, [-1, lstm_state_size[1]]), weight), bias))     # fc_layer_output.shape = [batch_size, num_fc_layer_units]

    with tf.variable_scope("final_fc_layer", reuse=tf.AUTO_REUSE):
        weight = tf.get_variable(name='w', shape=[hp.num_fc_layer_units, hp.num_outputs])
        bias = tf.get_variable(name='b', shape=[hp.num_outputs], initializer=tf.constant_initializer(0.0))
    logits = tf.add(tf.matmul(fc_layer_output, weight), bias)     # logits.shape = [batch_size, num_outputs]

    return logits


def gradient_clip(gradients, max_gradient_norm):
    clipped_gradients, gradient_norm = tf.clip_by_global_norm(gradients, max_gradient_norm)
    gradient_norm_summary = [tf.summary.scalar("grad_norm", gradient_norm)]
    gradient_norm_summary.append(tf.summary.scalar("clipped_gradient", tf.global_norm(clipped_gradients)))
    return clipped_gradients, gradient_norm_summary, gradient_norm


def train_network(train_x, train_y, test_x, test_y):
    start_time = time.time()
    network_input = tf.placeholder(dtype=tf.float32, shape=[None, hp.window_size, hp.feature_size], name='input_placeholder')
    network_output = tf.placeholder(dtype=tf.float32, shape=[None, hp.num_outputs], name='label_placeholder')

    num_training_samples = train_x.shape[0]

    logits = core_model(hp.lstm_state_size, network_input, hp.keep_prob_)    # logits.shape = [batch_size, num_outputs]

    total_loss = tf.losses.mean_squared_error(labels=network_output, predictions=tf.reshape(logits, [-1, hp.num_outputs]))
    optimizer = tf.train.AdamOptimizer(hp.learning_rate)

    with tf.name_scope("compute_gradients"):
        params = tf.trainable_variables(scope=None)
        grads = tf.gradients(ys=total_loss, xs=params, colocate_gradients_with_ops=True)    # optimizer.compute_gradients(loss)
        clipped_grads, grad_norm_summary, grad_norm = gradient_clip(grads, max_gradient_norm=hp.max_gradient_norm)
        grad_and_vars = zip(clipped_grads, params)
    
    global_step = tf.train.get_or_create_global_step()
    apply_gradient_op = optimizer.apply_gradients(grad_and_vars, global_step)

    # session_config = tf.ConfigProto(allow_soft_placement=True, log_device_placement=True)
    # session_config.gpu_options.allow_growth = True
    sess = tf.InteractiveSession()
    initializer = tf.contrib.layers.xavier_initializer()

    zero_row_indices = np.where(train_x.any(axis=1) == 0)[0]

    with tf.variable_scope(tf.get_variable_scope(), initializer=initializer): 
        sess.run(tf.global_variables_initializer())

        training_loss = []
        training_losses = []

        iteration_count = 1
        for epoch in range(hp.num_training_epochs):
            step = 0
            is_epoch_complete = False
            while is_epoch_complete == False:
                [batch_x, batch_y, step, is_epoch_complete] = get_next_full_batch(train_x, train_y, step, zero_row_indices)
                _, loss_val = sess.run([apply_gradient_op, total_loss], feed_dict={network_input:batch_x, network_output:batch_y})
                training_loss.append(loss_val)
                if iteration_count%10 == 0:
                    mean_training_loss = np.mean(training_loss)
                    print("Epoch :", epoch, "iteration :", iteration_count, "Loss Value :", mean_training_loss)
                    training_losses.append(mean_training_loss)
                    training_loss = []
                iteration_count += 1

        # Testing the network
        test_data_x = np.reshape(np.copy(test_x[0:hp.window_size]), (1, hp.window_size, hp.feature_size))

        pursuer_trajectory = []
        evader_trajectory = []

        for i in range(hp.window_size):
            pursuer_trajectory.append(np.reshape(np.copy(test_x[i,0:2]), (2,1)))
            evader_trajectory.append(np.reshape(np.copy(test_x[i,2:6]), (4,1)))

        for index in range(25):
            pursuer_next_position = np.reshape(sess.run([logits], feed_dict={network_input:test_data_x}), (2,1))
            evader_next_position = compute_evader_position(pursuer_next_position, pursuer_trajectory[index+hp.window_size-1], 
                evader_trajectory[index+hp.window_size-1], ti=1.0, vemax_repulsion=test_data_x[0,0,10], vemax_attraction=0, K=1.0)

            test_data_x[0, 0:hp.window_size-1, :] = np.copy(test_data_x[0, 1:hp.window_size, :])
            test_data_x[0, hp.window_size-1, 0:2] = np.reshape(np.copy(pursuer_next_position), (2))
            test_data_x[0, hp.window_size-1, 2:6] = np.reshape(np.copy(evader_next_position), (4))

            pursuer_trajectory.append(pursuer_next_position)
            evader_trajectory.append(evader_next_position)
            # index += 1
            # if np.linalg.norm(np.reshape(evader_next_position[0:2,:],(2,1)) - np.array([[feature_dict['destination_x']],[feature_dict['destination_y']]])) < 0.1 and \
            #     np.linalg.norm(np.reshape(evader_next_position[2:4,:],(2,1)) - np.array([[feature_dict['destination_x']],[feature_dict['destination_y']]])) < 0.1:
            #     metric = False

        pursuer_trajectory = np.array(pursuer_trajectory)
        pursuer_trajectory = np.transpose(pursuer_trajectory[:,:,0])  # (2,31)
        evader_trajectory = np.array(evader_trajectory)
        evader_trajectory = np.transpose(evader_trajectory[:,:,0])    # (4,31)

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


if __name__ == '__main__':
    data_file_path = 'dataset.csv'
    train_x, train_y, test_x, test_y = get_data(data_file_path)
    pursuer_trajectory, evader_trajectory = train_network(train_x, train_y, test_x, test_y)
    # print("Pursuer Trajectory :", pursuer_trajectory)
    # print("Evader Trajectory :", evader_trajectory)
    
    plt.plot(pursuer_trajectory[0,:], pursuer_trajectory[1,:], 'b', lw=1, marker='.')
    plt.plot(evader_trajectory[0,:], evader_trajectory[1,:], 'y', lw=1, marker='.')
    plt.plot(evader_trajectory[2,:], evader_trajectory[3,:], 'y', lw=1, marker='.')
    plt.grid(True)
    plt.show()

    # check_compute_evader_function(data_file_path)

    # testing the network
    # test_data_x = np.reshape(np.array([-1,-1,-0.5,0,0.5,0,1.0,1.0,0.3,0.05,0.4,0.05]), (1, 1, 12))
    # print(test_network(model_save_path, test_data_x))