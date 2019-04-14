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
    num_samples = data.shape[0]
    batch_partition_length = num_samples//hp.batch_size
    data_x = np.zeros([hp.batch_size, batch_partition_length, hp.feature_size], dtype=np.float32)
    data_y = np.zeros([hp.batch_size, batch_partition_length, hp.num_outputs], dtype=np.float32)

    for row in range(hp.batch_size):
        data_x[row,:,:] = np.reshape(data[row*batch_partition_length:(row+1)*batch_partition_length, 0:12], (1,batch_partition_length,hp.feature_size))
        data_y[row,:,:] = np.reshape(data[row*batch_partition_length:(row+1)*batch_partition_length, 12:14], (1,batch_partition_length,hp.num_outputs))

    # return data
    return data_x, data_y

def get_next_batch(data_x, data_y, index):
      return data_x[:,index*hp.window_size:(index+1)*hp.window_size,:], data_y[:,index*hp.window_size:(index+1)*hp.window_size,:]

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

def build_lstm_network(lstm_state_size, rnn_inputs, keep_prob_, batch_size):
    lstms = [tf.contrib.rnn.LayerNormBasicLSTMCell(num_units=size) for size in lstm_state_size]
    # Add dropout to the cell
    dropout_lstms = [tf.nn.rnn_cell.DropoutWrapper(cell=lstm, output_keep_prob=keep_prob_) for lstm in lstms]
    # Stack up multiple LSTM layers, for deep learning
    final_cell = tf.nn.rnn_cell.MultiRNNCell(dropout_lstms)
    # Getting an initial state of all zeros
    initial_state = final_cell.zero_state(batch_size, dtype=tf.float32)
    lstm_outputs, final_state = tf.nn.dynamic_rnn(final_cell, rnn_inputs, initial_state=initial_state)
    # print(lstm_outputs)
    # print(final_state)
    return lstm_outputs, final_state

def core_model(lstm_state_size, inputs, keep_prob_, batch_size):
    lstm_outputs, final_state = build_lstm_network(lstm_state_size, inputs, keep_prob_, batch_size)
    lstm_outputs = lstm_outputs[:,-1,:]        # taking the final state output
    # prev_layer = lstm_outputss
    # fc_size = hparams.fc_size
    
    with tf.variable_scope('out_predictions', reuse=tf.AUTO_REUSE):
        weight = tf.get_variable(name='w', shape=[hp.state_size, hp.num_outputs])
        bias = tf.get_variable(name='b', shape=[hp.num_outputs], initializer=tf.constant_initializer(0.0))
    logits = tf.add(tf.matmul(tf.reshape(lstm_outputs, [-1, hp.state_size]), weight), bias)

    # next_layer = []
    # for l in range(len(fc_size)):
    #     next_layer = utils.fully_connected(fc_size[l], prev_layer, 'FC_' + str(l))
    #     prev_layer = next_layer

    # logits = next_layer
    # logits = apply_dense_layer(outputs)
    # print(logits)
    return logits

def train_network(data_x, data_y, num_epochs=5):
    start_time = time.time()
    x = tf.placeholder(dtype=tf.float32, shape=[None, None, hp.feature_size], name='input_placeholder')
    y = tf.placeholder(dtype=tf.float32, shape=[None, None, hp.num_outputs], name='label_placeholder')
    init_state = tf.placeholder(dtype=tf.float32, shape=[None, hp.state_size], name='initial_state')

    batch_partition_length = data_x.shape[1]

    logits = core_model(hp.lstm_state_size, x, hp.keep_prob_, hp.batch_size)
    print("Logits are here :", logits)
    # rnn_inputs = x
    # cell = tf.contrib.rnn.BasicRNNCell(hp.state_size)
    # rnn_outputs, final_state = tf.nn.dynamic_rnn(cell, rnn_inputs, initial_state=init_state)   # output[:,-1,:] = final_state[:,:]
    # # rnn_outputs.shape = [hp.batch_size,hp.window_size,hp.state_size]

    # with tf.variable_scope('out_predictions', reuse=tf.AUTO_REUSE):
    #     weight = tf.get_variable(name='w', shape=[hp.state_size, hp.num_outputs])
    #     bias = tf.get_variable(name='b', shape=[hp.num_outputs], initializer=tf.constant_initializer(0.0))
    # logits = tf.add(tf.matmul(tf.reshape(rnn_outputs, [-1, hp.state_size]), weight), bias)

    total_loss = tf.losses.mean_squared_error(labels=y, predictions=tf.reshape(logits, [-1, hp.window_size, hp.num_outputs]))
    train_step = tf.train.AdamOptimizer(hp.learning_rate).minimize(total_loss)

    # global saver
    saver = tf.train.Saver()

    training_loss = []
    training_losses = []
    
    # with tf.Session() as sess:
    #     sess.run(tf.global_variables_initializer())
    #     epoch_size = batch_partition_length//hp.window_size

    #     iteration_count = 1
    #     for epoch in range(num_epochs):
    #         for index in range(epoch_size):
    #             [batch_x, batch_y] = get_next_batch(data_x, data_y, index)
    #             _, loss_val = sess.run([train_step, total_loss], feed_dict={x:batch_x, y:batch_y, init_state:np.zeros((hp.batch_size, hp.state_size))})
    #             training_loss.append(loss_val)
    #             if iteration_count%10 == 0:
    #                 mean_training_loss = np.mean(training_loss)
    #                 print("Epoch :", epoch, "iteration :", iteration_count, "Loss Value :", mean_training_loss)
    #                 training_losses.append(mean_training_loss)
    #                 training_loss = []
    #             iteration_count += 1
    #     # root_dir = os.getcwd()
    #     # save_path = saver.save(sess, root_dir+'/'+'model/mytrained_network.ckpt')
    #     # print("Model saved in path: %s" % save_path)
        
    #     feature_dict = {'pursuer_position_x':-1, 'pursuer_position_y':-1, 'evader_position_1x':-0.5, 'evader_position_1y':0,
    #     'evader_position_2x':0.5, 'evader_position_2y':0, 'destination_x':1.0, 'destination_y':1.0, 'vpmax':0.3, 'vpmin':0.05,
    #     'vemax_repulsion':0.4, 'epsilon':0.05}
    #     next_state = np.zeros((1, hp.state_size))

    #     test_data_x = np.reshape(np.array(list(feature_dict.values())), (1, 1, 12))
        
    #     pursuer_initial_position = np.reshape([feature_dict['pursuer_position_x'], feature_dict['pursuer_position_y']], (2,1))
    #     evader_initial_position = np.reshape([feature_dict['evader_position_1x'], feature_dict['evader_position_1y'], 
    #         feature_dict['evader_position_2x'], feature_dict['evader_position_2y']], (4,1))
        
    #     pursuer_trajectory = [pursuer_initial_position]
    #     evader_trajectory = [evader_initial_position]
        
    #     metric = True
    #     # index = 0
    #     for index in range(60):
    #     # while metric:
    #         [pursuer_next_position, next_state] = sess.run([logits, final_state], feed_dict={x:test_data_x, init_state:next_state})
    #         pursuer_next_position = np.reshape(pursuer_next_position, (2,1))

    #         evader_next_position = compute_evader_position(pursuer_next_position, pursuer_trajectory[index], evader_trajectory[index], 1.0, 
    #             feature_dict['vemax_repulsion'], 0, 1.0)

    #         feature_dict['pursuer_position_x'] = pursuer_next_position[0,0]
    #         feature_dict['pursuer_position_y'] = pursuer_next_position[1,0]
    #         feature_dict['evader_position_1x'] = evader_next_position[0,0]
    #         feature_dict['evader_position_1y'] = evader_next_position[1,0]
    #         feature_dict['evader_position_2x'] = evader_next_position[2,0]
    #         feature_dict['evader_position_2y'] = evader_next_position[3,0]

    #         test_data_x = np.reshape(np.array(list(feature_dict.values())), (1,1,12))
    #         pursuer_trajectory.append(pursuer_next_position)
    #         evader_trajectory.append(evader_next_position)
    #         # index += 1
    #         # if np.linalg.norm(np.reshape(evader_next_position[0:2,:],(2,1)) - np.array([[feature_dict['destination_x']],[feature_dict['destination_y']]])) < 0.1 and \
    #         #     np.linalg.norm(np.reshape(evader_next_position[2:4,:],(2,1)) - np.array([[feature_dict['destination_x']],[feature_dict['destination_y']]])) < 0.1:
    #         #     metric = False

    # pursuer_trajectory = np.array(pursuer_trajectory)
    # # print(pursuer_trajectory, pursuer_trajectory.shape)
    # pursuer_trajectory = np.transpose(pursuer_trajectory[:,:,0])  # (2,31)
    # # print(pursuer_trajectory, pursuer_trajectory.shape)
    # evader_trajectory = np.array(evader_trajectory)
    # # print(evader_trajectory, evader_trajectory.shape)
    # evader_trajectory = np.transpose(evader_trajectory[:,:,0])    # (4,31)
    # # print(evader_trajectory, evader_trajectory.shape)
    # end_time = time.time()
    # print("Training time : ", end_time - start_time)
    # return pursuer_trajectory, evader_trajectory

# def test_network(model_save_path, test_data_x):
#     with tf.Session() as sess:
#         global saver
#         saver.restore(sess, model_save_path)
#         pursuer_next_position = sess.run(logits, feed_dict={x:test_data_x})
#     return pursuer_next_position

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
    train_x, train_y = get_data(data_file_path)
    pursuer_trajectory, evader_trajectory = train_network(train_x, train_y, num_epochs=hp.num_training_epochs)
    
    # plt.plot(pursuer_trajectory[0,:],pursuer_trajectory[1,:])
    # plt.plot(evader_trajectory[0,:],evader_trajectory[1,:])
    # plt.plot(evader_trajectory[2,:],evader_trajectory[3,:])
    # plt.grid(True)
    # plt.show()

    # check_compute_evader_function(data_file_path)

    # testing the network
    # test_data_x = np.reshape(np.array([-1,-1,-0.5,0,0.5,0,1.0,1.0,0.3,0.05,0.4,0.05]), (1, 1, 12))
    # print(test_network(model_save_path, test_data_x))