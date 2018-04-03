
nodes = ["hyq","sphere_CoM"]
nodes = ["hyq","Vec_Acceleration","Vec_Velocity"]
nodes = ["hyq"]
r.client.gui.setCaptureTransform("/local/dev_hpp/screenBlender/iros2018/yaml/hyq_side_poly.yaml",nodes)
r(q_init)
r.client.gui.captureTransformOnRefresh(True)
player.displayContactPlan(1)
r(q_goal)
r.client.gui.captureTransformOnRefresh(False)

nodes = ["hyq_trunk_large"]
nodes = ["hyq_trunk_large","Vec_Acceleration","Vec_Velocity"]
r.client.gui.setCaptureTransform("/local/dev_hpp/screenBlender/iros2017/yaml/detour_path_GEOM_ctc.yaml",nodes)
r(q_init)
r.client.gui.captureTransformOnRefresh(True)
pp.speed=1
pp(1)
r.client.gui.captureTransformOnRefresh(False)





nodes = ["hrp2_trunk_flexible","Vec_Acceleration","Vec_Velocity"]
r.client.gui.setCaptureTransform("/local/dev_hpp/screenBlender/iros2017/yaml/slope_ctc_path.yaml",nodes)
r(q_init)
r.client.gui.captureTransformOnRefresh(True)
pp.speed=0.5
pp(2)
r.client.gui.captureTransformOnRefresh(False)




nodes = ["hrp2_14",'s']
nodes = ["hrp2_14"]
r.client.gui.setCaptureTransform("/local/dev_hpp/screenBlender/iros2018/yaml/darpa_contactSequence.yaml",nodes)
r(q_init)
r.client.gui.captureTransform()
r.client.gui.captureTransformOnRefresh(True)
player.displayContactPlan(1)
r(q_goal)
r.client.gui.captureTransformOnRefresh(False)


nodes = ['world/pinocchio']
gui.setCaptureTransform("/local/dev_hpp/screenBlender/iros2018/yaml/darpa_wholeBody.yaml",nodes)
#gui.captureTransform()
gui.captureTransformOnRefresh(True)
gui.captureTransformOnRefresh(False)



nodes = ['talos']
r.client.gui.setCaptureTransform("/local/dev_hpp/screenBlender/talos/yaml/airbus_stairs.yaml",nodes)
#r.client.gui.captureTransform()
r(configsFull[0])
r.client.gui.captureTransformOnRefresh(True)
displayContactSequence(r,configsFull,0.1)
r.client.gui.captureTransformOnRefresh(False)


r.client.gui.writeBlenderScript("/local/dev_hpp/screenBlender/talos/models/pyrene.py", ['talos'])

nodes = ["hrp2_14","Vec_Acceleration","Vec_Velocity"]
r.client.gui.setCaptureTransform("/local/dev_hpp/screenBlender/iros2017/yaml/slope_interp_3.yaml",nodes)
r.client.gui.captureTransformOnRefresh(True)
player.play(1/10.)
r.client.gui.captureTransformOnRefresh(False)


r.client.gui.writeNodeFile("path_0_root","/local/dev_hpp/screenBlender/iros2017/meshs/path_detour_geom_CTC.obj")





