
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
r.client.gui.setCaptureTransform("/local/dev_hpp/screenBlender/tro/yaml/darpa/success_cs.yaml",nodes)
r(q_init)
r.client.gui.captureTransform()
r.client.gui.captureTransformOnRefresh(True)
player.displayContactPlan(1)
r(q_goal)
r.client.gui.captureTransformOnRefresh(False)


nodes = ['world/pinocchio/visuals']
r.client.gui.setCaptureTransform("/local/dev_hpp/screenBlender/tro/yaml/darpa/motion2.yaml",nodes)
#gui.captureTransform()
r.client.gui.captureTransformOnRefresh(True)
r.client.gui.captureTransformOnRefresh(False)



nodes = ['talos']
r.client.gui.setCaptureTransform("/local/dev_hpp/screenBlender/talos/yaml/airbus_stairs.yaml",nodes)
#r.client.gui.captureTransform()
r(configsFull[0])
r.client.gui.captureTransformOnRefresh(True)
displayContactSequence(r,configsFull,0.1)
r.client.gui.captureTransformOnRefresh(False)


r.client.gui.writeBlenderScript("/local/dev_hpp/screenBlender/talos/models/pyrene.py", ['talos'])

r.client.gui.writeBlenderScript("/local/dev_hpp/screenBlender/talos/stairs.py", ['world'])

nodes = ["hrp2_14","Vec_Acceleration","Vec_Velocity"]
r.client.gui.setCaptureTransform("/local/dev_hpp/screenBlender/iros2017/yaml/slope_interp_3.yaml",nodes)
r.client.gui.captureTransformOnRefresh(True)
player.play(1/10.)
r.client.gui.captureTransformOnRefresh(False)


r.client.gui.writeNodeFile("path_0_root","/local/dev_hpp/screenBlender/iros2017/meshs/path_detour_geom_CTC.obj")


"""
l = end_effector_bezier_list['hrp2_lleg_rom'][3]

for i in range(len(l)):
  displayBezierCurve(r,l[i],offset = dict_offset['hrp2_lleg_rom'].translation.transpose().tolist()[0])
  
dir = "/local/dev_hpp/screenBlender/tro/curve/"
r.client.gui.writeNodeFile('path_73_LLEG_JOINT5',dir+"rrt.obj")
r.client.gui.writeNodeFile('bezier_curve_6',dir+"b0.obj")
r.client.gui.writeNodeFile('bezier_curve_7',dir+"b02.obj")
r.client.gui.writeNodeFile('bezier_curve_8',dir+"b04.obj")
r.client.gui.writeNodeFile('bezier_curve_9',dir+"b06.obj")
r.client.gui.writeNodeFile('bezier_curve_10',dir+"b08.obj")
r.client.gui.writeNodeFile('bezier_curve_11',dir+"b09.obj")
r.client.gui.writeNodeFile('bezier_curve_12',dir+"b1.obj")
"""

for i in range(6,10):
  r.client.gui.writeNodeFile("bezier_curve_"+str(i),dir+"b"+str(i)+".obj")

dir = "/local/dev_hpp/screenBlender/tro/curve/bar_xwp/"
for i in range(29):
  r.client.gui.writeNodeFile("path_"+str(i)+"_root",dir+"c_"+str(i)+".obj")

for i in range(23):
  r.client.gui.writeNodeFile("s"+str(i),dir+"s_"+str(i)+".stl")


dir = "/local/dev_hpp/screenBlender/tro/meshs/stones/darpa/fail2/"
for i in range(29):
  r.client.gui.writeNodeFile('stone_'+str(i),dir+'stone_'+str(i)+".stl")






