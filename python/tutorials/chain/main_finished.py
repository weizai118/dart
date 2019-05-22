import math
import numpy as np
import dartpy as dart


default_height = 1.0 # m
default_width = 0.2  # m
default_depth = 0.2  # m

default_torque = 15.0 # N-m
default_force = 15.0  # N
default_countdown = 200  # Number of timesteps for applying force

default_rest_position = 0.0
delta_rest_position = 10.0 * math.pi / 180.0

default_stiffness = 0.0
delta_stiffness = 10

default_damping = 5.0
delta_damping = 1.0


class Controller(object):
    def __init__(self, biped):
        self.biped
        self.dofs = self.biped.getNumDofs()

        self.forces = np.zeros(self.dofs)

        self.Kp = np.eye(self.dofs)
        self.Kd = np.eye(self.dofs)
        for i in range(6):
            self.Kp[i, i] = 0
            self.Kd[i, i] = 0
        for i in range(6, self.dofs):
            self.Kp[i, i] = 1000
            self.Kd[i, i] = 50

        self.target_positions = biped.getPositions()

    def set_target_positions(self, pose):
        self.target_positions = pose

    def clear_forces(self):
        self.forces.fill(0)


def make_root_body(chain, name):
    properties = dart.dynamics.BallJointProperties()


def add_body(chain, parent, name):
    pass


def main():
    # Create an empty Skeleton with the name 'chain'
    chain = dart.dynamics.Skeleton('chain')

    # Add each body to the last BodyNode in the pendulum
    bn = make_root_body(chain, 'body1')
    bn = add_body(chain, bn, 'body2')
    bn = add_body(chain, bn, 'body3')
    bn = add_body(chain, bn, 'body4')
    bn = add_body(chain, bn, 'body5')


if __name__ == "__main__":
    main()
