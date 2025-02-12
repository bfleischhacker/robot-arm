from build123d import *
from ocp_vscode import show_all, show, show_object, reset_show, set_port, set_defaults, get_defaults
from bd_warehouse import fastener
import copy
housing = import_step('resources/ST3215.step')
housing.label = 'housing'
RigidJoint('motor_gear', to_part=housing, joint_location=Location((-25.5, 8.4, 0), (90, 0, 0)))
show (housing, render_joints=True)