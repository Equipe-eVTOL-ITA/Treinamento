import random
import math
from constants import *


class FiniteStateMachine(object):
    """
    A finite state machine.
    """
    def __init__(self, state):
        self.state = state

    def change_state(self, new_state):
        self.state = new_state

    def update(self, agent):
        self.state.check_transition(agent, self)
        self.state.execute(agent)


class State(object):
    """
    Abstract state class.
    """
    def __init__(self, state_name):
        """
        Creates a state.

        :param state_name: the name of the state.
        :type state_name: str
        """
        self.state_name = state_name
        print("Entered", state_name)

    def check_transition(self, agent, fsm):
        """
        Checks conditions and execute a state transition if needed.

        :param agent: the agent where this state is being executed on.
        :param fsm: finite state machine associated to this state.
        """
        raise NotImplementedError("This method is abstract and must be implemented in derived classes")

    def execute(self, agent):
        """
        Executes the state logic.

        :param agent: the agent where this state is being executed on.
        """
        raise NotImplementedError("This method is abstract and must be implemented in derived classes")


class MoveForwardState(State):
    def __init__(self):
        super().__init__("MoveForward")
        self.time_passed = 0

    def check_transition(self, agent, state_machine):
        if agent.get_bumper_state():
            state_machine.change_state(GoBackState())
        elif self.time_passed > MOVE_FORWARD_TIME:
            state_machine.change_state(MoveInSpiralState())
        self.time_passed += SAMPLE_TIME

    def execute(self, agent):
        agent.set_velocity(FORWARD_SPEED, 0)


class MoveInSpiralState(State):
    def __init__(self):
        super().__init__("MoveInSpiral")
        self.time_passed = 0
    
    def check_transition(self, agent, state_machine):
        if agent.get_bumper_state():
            state_machine.change_state(GoBackState())
        elif self.time_passed > MOVE_IN_SPIRAL_TIME:
            state_machine.change_state(MoveForwardState())
        self.time_passed += SAMPLE_TIME

    def execute(self, agent):
        radius = INITIAL_RADIUS_SPIRAL + SPIRAL_FACTOR * self.time_passed
        angular_speed = FORWARD_SPEED / radius
        agent.set_velocity(FORWARD_SPEED, angular_speed)


class GoBackState(State):
    def __init__(self):
        super().__init__("GoBack")
        self.time_passed = 0

    def check_transition(self, agent, state_machine):
        if self.time_passed > GO_BACK_TIME:
            state_machine.change_state(RotateState())
        self.time_passed += SAMPLE_TIME

    def execute(self, agent):
        agent.set_velocity(BACKWARD_SPEED, 0) 


class RotateState(State):
    def __init__(self):
        super().__init__("Rotate")
        self.target_angle = random.uniform(0, 2 * math.pi)
        self.angle = 0.0

    def check_transition(self, agent, state_machine):
        self.angle = agent.pose.rotation % (2 * math.pi)

        if abs(self.angle - self.target_angle) <= ANGULAR_SPEED * SAMPLE_TIME:
            state_machine.change_state(MoveForwardState())
    
    def execute(self, agent):
        
        if self.angle < self.target_angle:
            agent.set_velocity (0, ANGULAR_SPEED)
        else:
            agent.set_velocity (0, -ANGULAR_SPEED)
