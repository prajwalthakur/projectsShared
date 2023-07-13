import os
import numpy as np
import tensorflow as tf
from tensorflow.initializers import random_uniform
from gym.spaces import Box


class Actor():
    def __init__(self, lr, n_actions, name, input_dims, sess, fc1_dims, fc2_dims,
                 act_space: Box, chkpt_dir, batch_size=64):
        self.lr = lr
        self.fc1_dims = fc1_dims
        self.fc2_dims = fc2_dims
        self.n_actions = n_actions
        self.name = name
        self.batch_size = batch_size
        # self.action_bound = action_bound
        self.batch_size = batch_size
        self.sess = sess
        self.input_dims = input_dims



        self.act_scale_t = tf.convert_to_tensor((act_space.high - act_space.low) / 2.,
                                                dtype=tf.float32)  # to scale the action output from -1...1 into the range from
        # low...high
        self.act_shift_t = tf.convert_to_tensor((act_space.high + act_space.low) / 2.,
                                                dtype=tf.float32)  # to scale the action output from -1...1 into the range from
        # low...high
        self.build_network()


        self.params = tf.trainable_variables(scope=self.name)
        self.saver = tf.train.Saver()
        self.checkpoint_dir = chkpt_dir


        self.unnormalized_actor_gradients = tf.gradients(self.mu, self.params, -self.action_gradient)
        self.normalized_actor_gradients = list(
            map(lambda x: tf.div(x, self.batch_size), self.unnormalized_actor_gradients))
        self.optimize = tf.train.AdamOptimizer(self.lr).apply_gradients(
            zip(self.normalized_actor_gradients, self.params))

    def build_network(self):
        with tf.variable_scope(self.name):
            self.input_var = tf.placeholder(tf.float32, shape=[None, *self.input_dims], name='inputs')
            self.action_gradient = tf.placeholder(tf.float32, shape=[None, self.n_actions])
            f1 = 1 / np.sqrt(self.fc1_dims)
            dense1 = tf.layers.dense(self.input_var, units=self.fc1_dims,
                                     kernel_initializer=random_uniform(-f1, f1), bias_initializer=
                                     random_uniform(-f1, f1))
            batch1 = tf.layers.batch_normalization(dense1)
            layer1_activation = tf.nn.relu(batch1)

            f2 = 1 / np.sqrt(self.fc2_dims)
            dense2 = tf.layers.dense(layer1_activation, units=self.fc2_dims,
                                     kernel_initializer=random_uniform(-f2, f2), bias_initializer=
                                     random_uniform(-f2, f2))
            batch2 = tf.layers.batch_normalization(dense2)
            layer2_activation = tf.nn.relu(batch2)
            f3 = 0.003  # from paper
            mu = tf.layers.dense(layer2_activation, units=self.n_actions, activation='tanh',
                                 kernel_initializer=random_uniform(-f3, f3),
                                 bias_initializer=random_uniform(-f3, f3))
            self.mu = tf.multiply(mu,
                                  self.act_scale_t) + self.act_shift_t  # TODO (done) actual scaling and adding biasing

    def predict(self, inputs):
        return self.sess.run(self.mu, feed_dict={self.input_var: inputs})

    def train(self, inputs, gradients):
        self.sess.run(self.optimize, feed_dict={self.input_var: inputs, self.action_gradient: gradients})

    def save_checkpoint(self, i):
        print("...saving actor checkpoint...")
        self.checkpoint_file = os.path.join(self.checkpoint_dir, self.name + str(i) +  '_ddpg.ckpt')
        self.saver.save(self.sess, self.checkpoint_file)

    def load_checkpoint(self,i):
        print("...loading actor checkpoint...")
        self.checkpoint_file = os.path.join(self.checkpoint_dir, self.name + str(i) +  '_ddpg.ckpt')
        self.saver.restore(self.sess, self.checkpoint_file)


class Critic(object):
    def __init__(self, lr, n_actions, name, input_dims, sess, fc1_dims, fc2_dims,chkpt_dir , batch_size=64,
                 ):
        super().__init__()
        self.lr = lr
        self.fc1_dims = fc1_dims
        self.fc2_dims = fc2_dims
        self.n_actions = n_actions
        self.name = name
        self.batch_size = batch_size
        self.batch_size = batch_size
        self.sess = sess
        self.input_dims = input_dims

        self.build_network()
        self.params = tf.trainable_variables(scope=self.name)
        self.saver = tf.train.Saver()
        self.checkpoint_dir = chkpt_dir
        # self.checkpoint_file = os.path.join(self.checkpoint_dir, self.name + '_ddpg.ckpt')
        self.optimize = tf.train.AdamOptimizer(learning_rate = self.lr).minimize(self.loss)
        self.action_gradient = tf.gradients(self.q, self.actions)

    def build_network(self):
        with tf.variable_scope(self.name):
            self.input_var = tf.placeholder(tf.float32, shape=[None, *self.input_dims], name='inputs')
            self.actions = tf.placeholder(tf.float32, shape=[None, self.n_actions], name='actions')
            self.q_target = tf.placeholder(tf.float32, shape=[None, 1], name='targets')
            f1 = 1 / np.sqrt(self.fc1_dims)
            dense1 = tf.layers.dense(self.input_var, units=self.fc1_dims,
                                     kernel_initializer=random_uniform(-f1, f1), bias_initializer=
                                     random_uniform(-f1, f1))
            batch1 = tf.layers.batch_normalization(dense1)
            layer1_activation = tf.nn.relu(batch1)

            f2 = 1 / np.sqrt(self.fc2_dims)
            dense2 = tf.layers.dense(layer1_activation, units=self.fc2_dims,
                                     kernel_initializer=random_uniform(-f2, f2), bias_initializer=
                                     random_uniform(-f2, f2))
            batch2 = tf.layers.batch_normalization(dense2)
            action_in = tf.layers.dense(self.actions, units=self.fc2_dims, activation='relu')

            state_actions = tf.add(batch2, action_in)
            #state_actions = batch2
            state_actions = tf.nn.relu(state_actions)

            f3 = 0.003  # from paper
            self.q = tf.layers.dense(state_actions, units=1,
                                     kernel_initializer=random_uniform(-f3, f3),
                                     bias_initializer=random_uniform(-f3, f3),
                                     kernel_regularizer=tf.keras.regularizers.l2(0.01))
            self.loss = tf.losses.mean_squared_error(self.q_target, self.q)

    def predict(self, inputs, actions):
        return self.sess.run(self.q, feed_dict={self.input_var: inputs, self.actions: actions})

    def train(self, inputs, actions, q_target):
        return self.sess.run(self.optimize,
                             feed_dict={self.input_var: inputs, self.actions: actions, self.q_target: q_target})

    def get_action_gradients(self, inputs, actions):
        return self.sess.run(self.action_gradient, feed_dict={self.input_var: inputs, self.actions: actions})

    def save_checkpoint(self,i):
        print("...saving critic checkpoint...")
        self.checkpoint_file = os.path.join(self.checkpoint_dir, self.name + str(i) +  '_ddpg.ckpt')
        self.saver.save(self.sess, self.checkpoint_file)

    def load_checkpoint(self,i):
        print("...loading critic checkpoint...")
        self.checkpoint_file = os.path.join(self.checkpoint_dir, self.name + str(i) +  '_ddpg.ckpt')
        self.saver.restore(self.sess, self.checkpoint_file)
