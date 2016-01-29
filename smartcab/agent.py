import random
import pickle
from environment import Agent, Environment
from planner import RoutePlanner
from simulator import Simulator

class LearningAgent(Agent):
    """An agent that learns to drive in the smartcab world."""

    def __init__(self, env):
        super(LearningAgent, self).__init__(env)  # sets self.env = env, state = None, next_waypoint = None, and a default color
        self.color = 'red'  # override color
        self.planner = RoutePlanner(self.env, self)  # simple route planner to get next_waypoint
        # TODO: Initialize any additional variables here
        self.q = dict()
        self.actions = [None, 'forward', 'left', 'right']
        self.epsilon=0.5
        self.alpha = 0.15
        self.success = 0
        self.update_nb = 0
        self.update_list = []

    def reset(self, destination=None):
        self.planner.route_to(destination)
        # TODO: Prepare for a new trip; reset any variables here, if required

    def update(self, t):
        # Gather inputs
        self.next_waypoint = self.planner.next_waypoint()  # from route planner, also displayed by simulator
        inputs = self.env.sense(self)
        inputs = inputs.items()
        deadline = self.env.get_deadline(self)

        # TODO: Update state
        self.state = (inputs[0],inputs[1],inputs[2],inputs[3],self.next_waypoint)
        
        # TODO: Select action according to your policy
        agent_policy = 'not basic'


        rd_action = random.choice(self.actions)

        # TODO: Learn policy based on state, action, reward

        def getQ(self, state, action):
            return self.q.get((state, action))

        def learnQ(self, state, action, reward):
            oldv = self.q.get((state, action), None)
            if oldv is None:
                self.q[(state, action)] = reward
            else:
                self.q[(state, action)] = oldv + self.alpha * (reward - oldv)

            #self.update_nb += 1
            #self.update_list.append([{(state, action):(self.q[(state, action)])},self.update_nb])
            #file_object = open('q_values_e_{0}'.format(0.8),'wb')
            #pickle.dump(self.update_list,file_object)
            #file_object.close()

        def chooseAction(self, state):
            q = [getQ(self,state, a) for a in self.actions]
            maxQ = max(q)
            print q

            if random.random() < self.epsilon:
                action = random.choice(self.actions)
                self.epsilon = 0.99 * self.epsilon # epsilon decays over time
                print "epsilon"
                print self.epsilon

            else:
              count = q.count(maxQ)
              if count > 1:
                  best = [i for i in range(len(self.actions)) if q[i] == maxQ]
                  i = random.choice(best)
              else:
                  i = q.index(maxQ)

              action = self.actions[i]
            print 'Exploration of state space is {0:.2f} %'.format((float(len(self.q.keys())) / 512) * 100)

            return action
        
        if agent_policy == 'basic':
          action = rd_action
        else:
          action = chooseAction(self,self.state)
        # Execute action and get reward
        reward = self.env.act(self, action)
        if agent_policy != 'basic':
          learnQ(self,self.state,action,reward)

        if reward > 10:
            self.success += 1
            print "Destination reached {0} times".format(self.success)

        print "LearningAgent.update(): deadline = {}, inputs = {}, action = {}, reward = {}".format(deadline, inputs, action, reward)  # [debug]


def run():
    """Run the agent for a finite number of trials."""

    # Set up environment and agent
    e = Environment()  # create environment (also adds some dummy traffic)
    a = e.create_agent(LearningAgent)  # create agent
    e.set_primary_agent(a, enforce_deadline=True)  # set agent to track

    # Now simulate it
    sim = Simulator(e, update_delay=0.01)  # reduce update_delay to speed up simulation
    sim.run(n_trials=100)  # press Esc or close pygame window to quit


if __name__ == '__main__':
    run()
