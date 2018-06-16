import pinocchio as se3
from pinocchio import SE3, Quaternion

rleg_id = "rh_foot_joint"
lleg_id = "lh_foot_joint"
rhand_id = "rf_foot_joint"
lhand_id = "lf_foot_joint"
rleg_rom = 'rhleg'
lleg_rom = 'lhleg'
rhand_rom = 'rfleg'
lhand_rom = 'lfleg'


limbs_names = [rleg_rom,lleg_rom,rhand_rom,lhand_rom]

# FIXME : retrive value from somewhere ? 
MRsole_offset = se3.SE3.Identity()
rot = np.matrix([[0.,0,1],[0.,1.,0.],[-1.,0.,0.]])
MRsole_offset.rotation = rot
MLsole_offset = se3.SE3.Identity()
rot = np.matrix([[0.,0,1],[0.,1.,0.],[-1.,0.,0.]])
MLsole_offset.rotation = rot
MRhand_offset = se3.SE3.Identity()
rot = np.matrix([[0.,0,1],[0.,1.,0.],[-1.,0.,0.]])
MRhand_offset.rotation = rot
MLhand_offset = se3.SE3.Identity()
rot = np.matrix([[0.,0,1],[0.,1.,0.],[-1.,0.,0.]])
MLhand_offset.rotation = rot

# display transform :

MRsole_display = se3.SE3.Identity()
MLsole_display = se3.SE3.Identity()
MRhand_display = se3.SE3.Identity()
MLhand_display = se3.SE3.Identity()


from generate_contact_sequence import *