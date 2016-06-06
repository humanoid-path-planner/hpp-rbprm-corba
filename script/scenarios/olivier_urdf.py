from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
import hpp.corbaserver.rbprm.tools.plot_analytics  as plot_ana

def loadRobot(packageName,meshPackageName,rootJointType,urdfName,urdfSuffix,srdfSuffix, limbId, limbRoot, limbEffector):
	fullBody = FullBody ()
	fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
	fullBody.setJointBounds ("base_joint_xyz", [0,0,0,0,0,0])

	limbOffset = [0,0,0] #inutile ici
	limbNormal = [0,1,0] #inutile ici
	limbx = 0.09; limby = 0.05 #inutile ici
	fullBody.addLimb(limbId,limbRoot,'',limbOffset,limbNormal, limbx, limby, 1000, "manipulability", 0.1)
	return fullBody

def runall(lid, valueNames):	
	res = {} #dictionnaire des values min / max pour chaque critere
	#~ fullBody.runLimbSampleAnalysis(lid, "jointLimitsDistance", False)
	for valueName in valueNames:
		test = fullBody.runLimbSampleAnalysis(lid, valueName, False)
		res [valueName] = test
	return res

def retrieveOctreeValues(fullBody, valueNames,limbId):
	res = {}	
	for valueName in valueNames:
		res [valueName] = plot_ana.getOctreeValues(fullBody, valueName, limbId)
	return res
	
def rescaleOctreeValue(valueName, robotData):
	r1_min = robotData[0]["valueBounds"][valueName][0]
	r2_min = robotData[1]["valueBounds"][valueName][0]
	r1_max = robotData[0]["valueBounds"][valueName][1]
	r2_max = robotData[1]["valueBounds"][valueName][1]
	r_min = min(r1_min, r2_min)
	r_max = max(r1_max, r2_max)
	
	for i in range(0,len(robotData[0]["octreeValues"][valueName]['values'])):
		val = robotData[0]["octreeValues"][valueName]['values'][i]
		robotData[0]["octreeValues"][valueName]['values'][i] = ((val * r1_max + r1_min) - r_min) / r_max
		
	for i in range(0,len(robotData[1]["octreeValues"][valueName]['values'])):
		val = robotData[1]["octreeValues"][valueName]['values'][i]
		robotData[1]["octreeValues"][valueName]['values'][i] = ((val * r2_max + r2_min) - r_min) / r_max
	
def rescaleOctreeValues(valueNames, robotData):
	for valueName in valueNames:
		rescaleOctreeValue(valueName, robotData)

#define values to analyze #EDIT
valueNames = [
#~ "isotropy", 	#whole jacobian
#~ "isotropyRot", 	#rotation jacobian
#~ "isotropyTr", 	#translation jacobian
"minimumSingularValue",
"minimumSingularValueRot",
"minimumSingularValueTr",
"maximumSingularValue",
"maximumSingularValueRot",
"maximumSingularValueTr",
"manipulabilityRot",
"manipulabilityTr",
"manipulability"
]

robotData = [{},{}]

#first load first robot data
packageName = "laas_design" #EDIT
meshPackageName = "laas_design" #EDIT
rootJointType = "freeflyer" #EDIT

#  Information to retrieve urdf and srdf files.
urdfName = "laas_arm_wrist_ZXZ" #EDIT
urdfSuffix = "" #EDIT
srdfSuffix = "" #EDIT

limbId = '0' #nom que tu souhaites, peu importe #EDIT
limbRoot = 'shoulder_joint' #joint racine de la chaine a analyser #EDIT
limbEffector = '' # joint qui correspond a l'effecteur, laisse vide si dernier joint #EDIT
fullBody = loadRobot(packageName,meshPackageName,rootJointType,urdfName,urdfSuffix,srdfSuffix, limbId, limbRoot, limbEffector)

# run analysis
limbValueBounds =  runall(limbId, valueNames)

# compute normalized octree cube values
robotData[0]["octreeValues"] = retrieveOctreeValues(fullBody, valueNames, limbId)
robotData[0]["valueBounds"] = limbValueBounds
robotData[0]["name"] = urdfName + limbId

#now to the second robot

packageName = "laas_design" #EDIT
meshPackageName = "laas_design" #EDIT
rootJointType = "freeflyer" #EDIT

#  Information to retrieve urdf and srdf files.
urdfName = "laas_arm_wrist_ZXY" #EDIT
urdfSuffix = "" #EDIT
srdfSuffix = "" #EDIT

limbId = '1' #EDIT
limbRoot = 'shoulder_joint' #EDIT
limbEffector = '' #EDIT
fullBody = loadRobot(packageName,meshPackageName,rootJointType,urdfName,urdfSuffix,srdfSuffix, limbId, limbRoot, limbEffector)

# run analysis
limbValueBounds =  runall(limbId, valueNames)
# compute normalized octree cube values
robotData[1]["octreeValues"] = retrieveOctreeValues(fullBody, valueNames, limbId)
robotData[1]["valueBounds"] = limbValueBounds
robotData[1]["name"] = urdfName + limbId

rescaleOctreeValues(valueNames, robotData)
for valueName in valueNames:
	plot_ana.compareOctreeValues(robotData[0]["name"], robotData[1]["name"], robotData[0]["octreeValues"][valueName], robotData[1]["octreeValues"][valueName], valueName)
	
import matplotlib.pyplot as plt
plt.show()
