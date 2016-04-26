import numpy
import matplotlib.pyplot as plt
import pybrain
from pybrain.tools.shortcuts import buildNetwork
from pybrain.rl.agents import LearningAgent
from pybrain.rl.learners import Q, ActionValueNetwork
from pybrain.rl.experiments import ContinuousExperiment
from pybrain.rl.environments import Environment
from pybrain.rl.environments import Task
from pybrain.rl.explorers import NormalExplorer


if __name__ == "__main__":
    print "hey fellas"

    # continuous environment (state of the system is R^4+, so a table's size would explode)
    # could we have continuous actions as well?
    #    we could have a finite set of motor voltages or increments in voltage,
    #    Or, since we are already using a NN for Q, why not just input the voltage directly?

    """
    need to learn what each part of pybrain.rl does
        agent
          An agent is an entity capable of producing actions,
            based on previous observations.
          Generally it will also learn from experience.
          It can interact directly with a Task.

          LearningAgent has a module, a learner that modifies the module,
            and an explorer which perturbs the actions.
          It can have learning enabled or disabled and can be used continuously
            or with episodes.

        learners
          The top of the learner hierarchy is more conceptual than functional.
          The different classes distinguish algorithms in such a way that we can
            automatically determine when an algorithm is not applicable for a problem.
          Q - value based learner

        experiments
          An experiment matches up a task with an agent and handles their interactions.
          ContinuousExperiment - handles continuous tasks

        tasks
          A task is associating a purpose with an environment. It decides how to
          evaluate the observations, potentially returning reinforcement rewards or
          fitness values. Furthermore it is a filter for what should be visible
          to the agent. Also, it can potentially act as a filter on how actions are
          transmitted to the environment.

        explorer
          An Explorer object is used in Agents, receives the current state and
          action (from the controller Module) and returns an explorative action that
          is executed instead of the given action.
          Normal Explorer - a continuos explorer, perturbs actions with noise

        module
          An input-output layer with utility such that it can be a layer in a NN

        environment

    we will probably have to make our own environment with ODE integration that
      represents the dynamics of the knee we are trying to model

    our goal is to track a time series of desired torques with minimal effort (or
      opposing effort)


    """