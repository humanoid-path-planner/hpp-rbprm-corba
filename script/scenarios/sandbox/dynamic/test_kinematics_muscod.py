# use this script after a contact sequence AND a trajectory from muscod has been loaded

#from hpp.corbaserver.rbprm.tools.obj_to_constraints import *
from hpp.corbaserver.rbprm.tools.com_constraints import *




limbsCOMConstraints = { rLegId : {'file': "hrp2/RL_com.ineq", 'effector' : 'RLEG_JOINT5'},  
                        lLegId : {'file': "hrp2/LL_com.ineq", 'effector' : 'LLEG_JOINT5'},
                        rarmId : {'file': "hrp2/RA_com.ineq", 'effector' : rHand},
                        larmId : {'file': "hrp2/LA_com.ineq", 'effector' : lHand} }

#DEFINE state and config: 

state1=1
state2=2

config1=fullBody.getConfigAtState(state1)
config2=fullBody.getConfigAtState(state2)


#res = get_com_constraint(fullBody, state, config, limbsCOMConstraints, interm = False, exceptList = [])
res1 = get_com_constraint(fullBody, state1, config1, limbsCOMConstraints, interm = False)
res1Bis = get_com_constraint(fullBody, state1, config1, limbsCOMConstraints, interm = True)
res2Bis = get_com_constraint(fullBody, state2, config2, limbsCOMConstraints, interm = True)
res2 = get_com_constraint(fullBody, state2, config2, limbsCOMConstraints, interm = False)



# DEFINE com :

com1 = c1[0].tolist()[0]
com1Bis = c2[0].tolist()[0]
com2Bis = c3[0].tolist()[0]   
com2 = c3[-1].tolist()[0]



value = np.max(res1[0].dot(com1) - res1[1])
print("value 1 = "+str(value))
if value > 0. :
    print("Infeasible.")


value = np.max(res1Bis[0].dot(com1Bis) - res1Bis[1])
print("value 1 bis = "+str(value))
if value > 0. :
    print("Infeasible.")

value = np.max(res2Bis[0].dot(com2Bis) - res2Bis[1])
print("value 2 bis = "+str(value))
if value > 0. :
    print("Infeasible.")
    
value = np.max(res2[0].dot(com2) - res2[1])
print("value 2 = "+str(value))
if value > 0. :
    print("Infeasible.")            