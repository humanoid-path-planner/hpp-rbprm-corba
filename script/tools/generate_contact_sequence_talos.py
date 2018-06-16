
import pinocchio as se3
from pinocchio import SE3, Quaternion

rleg_id = "leg_right_sole_fix_joint"
lleg_id = "leg_left_sole_fix_joint"
rhand_id = "gripper_right_inner_single_joint"
lhand_id = "gripper_left_inner_single_joint"
rleg_rom = 'rleg'
lleg_rom = 'lleg'
rhand_rom = 'rarm'
lhand_rom = 'larm'
limbs_names = [rleg_rom,lleg_rom,rhand_rom,lhand_rom]

# FIXME : retrive value from somewhere ? (it's actually in generate_straight_walk_muscod)
MRsole_offset = se3.SE3.Identity()
#MRsole_offset.translation = np.matrix([0.0146,  -0.01, -0.105])
MLsole_offset = se3.SE3.Identity()
#MLsole_offset.translation = np.matrix([0.0146,  0.01, -0.105])
MRhand_offset = se3.SE3.Identity()
#rot = np.matrix([[0.,1.,0.],[1.,0.,0.],[0.,0.,-1.]])
#MRhand_offset.rotation = rot
MLhand_offset = se3.SE3.Identity()
#rot = np.matrix([[0.,1.,0.],[1.,0.,0.],[0.,0.,-1.]]) # TODO : check this
#MRhand_offset.rotation = rot

# display transform :

MRsole_display = se3.SE3.Identity()
MLsole_display = se3.SE3.Identity()
MRhand_display = se3.SE3.Identity()
#MRhand_display.translation = np.matrix([0,  0., -0.11])
MLhand_display = se3.SE3.Identity()
#MLhand_display.translation = np.matrix([0,  0., -0.11])

from generate_contact_sequence import *