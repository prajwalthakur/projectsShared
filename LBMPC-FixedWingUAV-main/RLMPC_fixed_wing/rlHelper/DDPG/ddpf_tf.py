# can find deep deterministic policy
# the policy is deterministic ,how to handle explore-exploit?
# will need a way to bound action to actual action the environment will take
# two actor and two critic netwrok , a target for each
# updates are soft ,theta' = tau*theta + (1-tau)*theta'
# the target actor is the just the evaluation actor + some noise process
# Used Ornstein Uhlenbeck temporarly corelated from previous state ->will need a class for noise
# batch normalization


import numpy as np
import tensorflow as tf
from rlHelper.DDPG.networks import Actor, Critic
from rlHelper.DDPG.ReplayBuffer import ReplayBuffer
from rlHelper.DDPG.OUNoise import OUActionNoise


class Agent:
    def __init__(self, alpha, beta, input_dims, tau, env, logdir, gamma=0.99, n_actions=7,
                 max_size=600, layer1_size=400, layer2_size=300,
                 batch_size=32):
        self.gamma = gamma
        self.tau = tau
        self.memory = ReplayBuffer(max_size, input_dims, n_actions)
        self.batch_size = batch_size
        self.sess = tf.Session()
        self.actor = Actor(alpha, n_actions, 'Actor', input_dims, self.sess,
                           layer1_size, layer2_size, env.action_space, logdir)
        self.critic = Critic(beta, n_actions, 'Critic', input_dims, self.sess,
                             layer1_size, layer2_size, logdir)

        self.target_actor = Actor(alpha, n_actions, 'TargetActor',
                                  input_dims, self.sess, layer1_size,
                                  layer2_size, env.action_space, logdir)
        self.target_critic = Critic(beta, n_actions, 'TargetCritic', input_dims,
                                    self.sess, layer1_size, layer2_size, logdir)

        self.noise = OUActionNoise(mu=np.zeros(n_actions))

        # define ops here in __init__ otherwise time to execute the op
        # increases with each execution.
        self.update_critic = [self.target_critic.params[i].assign(
            tf.multiply(self.critic.params[i], self.tau) \
            + tf.multiply(self.target_critic.params[i], 1. - self.tau))
            for i in range(len(self.target_critic.params))]

        self.update_actor = \
            [self.target_actor.params[i].assign(
                tf.multiply(self.actor.params[i], self.tau) \
                + tf.multiply(self.target_actor.params[i], 1. - self.tau))
                for i in range(len(self.target_actor.params))]

        self.sess.run(tf.global_variables_initializer())

        self.update_network_parameters(first=True)
        self.itr = 0
        self.learnitr = 0

    def update_network_parameters(self, first=False):
        if first:
            old_tau = self.tau
            self.tau = 1.0
            self.target_critic.sess.run(self.update_critic)
            self.target_actor.sess.run(self.update_actor)
            self.tau = old_tau
        else:
            self.target_critic.sess.run(self.update_critic)
            self.target_actor.sess.run(self.update_actor)

    def remember(self, state, action, reward, new_state, done):
        self.memory.store_transition(state, action, reward, new_state, done)

    def choose_action(self, state):

        state = state[np.newaxis, :]
        mu = self.actor.predict(state)  # returns list of list
        noise = self.noise()
        mu_prime = mu + noise

        return mu_prime[0]

    def learn(self):

        if self.memory.mem_cntr < self.batch_size:
            flag = 0
            self.learnitr += 1
            return flag
        """if self.learnitr % self.batch_size != 0:
            self.learnitr += 1
            flag = 0
            return flag"""
        self.learnitr += 1
        flag = 1
        state, action, reward, new_state, done = \
            self.memory.sample_buffer(self.batch_size)

        critic_value_ = self.target_critic.predict(new_state,
                                                   self.target_actor.predict(new_state))
        target = []
        for j in range(self.batch_size):
            target.append(reward[j] + self.gamma * critic_value_[j] * done[j])
        target = np.reshape(target, (self.batch_size, 1))

        _ = self.critic.train(state, action, target)

        a_outs = self.actor.predict(state)
        grads = self.critic.get_action_gradients(state, a_outs)

        self.actor.train(state, grads[0])

        self.update_network_parameters()
        if self.itr%1000==0:
            print("saving model itr number=",self.itr)
            self.save_models(self.itr)
        #print("flag=", flag)
        self.itr += 1
        return flag

    def save_models(self, itr):
        self.actor.save_checkpoint(itr)
        self.target_actor.save_checkpoint(itr)
        self.critic.save_checkpoint(itr)
        self.target_critic.save_checkpoint(itr)

    def load_models(self, itr):
        self.actor.load_checkpoint(itr)
        self.target_actor.load_checkpoint(itr)
        self.critic.save_checkpoint(itr)
        self.target_critic.save_checkpoint(itr)
