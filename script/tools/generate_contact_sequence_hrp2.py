
import pinocchio as se3
from pinocchio import SE3, Quaternion

rleg_id = "RLEG_JOINT5"
lleg_id = "LLEG_JOINT5"
rhand_id = "RARM_JOINT5"
lhand_id = "LARM_JOINT5"
rleg_rom = 'hrp2_rleg_rom'
lleg_rom = 'hrp2_lleg_rom'
rhand_rom = 'hrp2_rarm_rom'
lhand_rom = 'hrp2_larm_rom'
limbs_names = [rleg_rom,lleg_rom,rhand_rom,lhand_rom]

# FIXME : retrive value from somewhere ? (it's actually in generate_straight_walk_muscod)
MRsole_offset = se3.SE3.Identity()
MRsole_offset.translation = np.matrix([0.0146,  -0.01, -0.105])
MLsole_offset = se3.SE3.Identity()
MLsole_offset.translation = np.matrix([0.0146,  0.01, -0.105])
MRhand_offset = se3.SE3.Identity()
rot = np.matrix([[0.,1.,0.],[1.,0.,0.],[0.,0.,-1.]])
MRhand_offset.rotation = rot
MLhand_offset = se3.SE3.Identity()
rot = np.matrix([[0.,1.,0.],[1.,0.,0.],[0.,0.,-1.]]) # TODO : check this
MRhand_offset.rotation = rot

# display transform :

MRsole_display = se3.SE3.Identity()
MLsole_display = se3.SE3.Identity()
MRhand_display = se3.SE3.Identity()
MRhand_display.translation = np.matrix([0,  0., -0.11])
MLhand_display = se3.SE3.Identity()
MLhand_display.translation = np.matrix([0,  0., -0.11])

from generate_contact_sequence import *