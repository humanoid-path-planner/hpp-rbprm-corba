from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer

import stair_bauzil_hrp2_interp as stair
import gen_hrp2_statically_balanced_positions_2d as gen

# ~
configs = stair.configs[3:6]
# ~ configs = [stair.configs[4]]

limbs = ["1lLeg", "0rLeg", "3Rarm"]
# ~ limbs = ['1lLeg', '0rLeg']
print("configs", len(configs))
all_states = gen.gen(
    limbs,
    num_samples=0,
    coplanar=False,
    num_candidates_per_config=10,
    num_contact_candidates=10,
    q_entries=configs,
    projectToObstacles=True,
)
