import subprocess
import os
DIR = "/local/fernbach/qhull/constraints_obj/"
STAB_NAME = "stability"
CONS_NAME = "constraints"
KIN_NAME = "kinematics"
BEZIER_NAME = "bezier_wp"


def generate_off_file(name):
    os.remove(DIR+name+"_.off") if os.path.isfile(DIR+name+"_.off") else None
    os.remove(DIR+name+".off") if os.path.isfile(DIR+name+".off") else None

    #cmd = "cat "+DIR+name+".txt | qhalf Fp | qconvex o >> "+DIR+name+"_.off"
    cmd = "cat "+DIR+name+".txt | qhalf FP | qconvex Ft  >> "+DIR+name+"_.off"
    try :
        subprocess.check_output(cmd,shell=True)
        print("qHull OK for file : "+name+".txt")
    except subprocess.CalledProcessError:
        print("Error in qHull for file "+name+".txt")
    #replace first line with "OFF"
    with open(DIR+name+"_.off") as inFile, open(DIR+name+".off","w") as outFile:
        for i,line in enumerate(inFile):
            if i == 0:
                outFile.write("OFF\n")
            else :
                outFile.write(line)
    inFile.close()
    outFile.close()
    os.remove(DIR+name+"_.off")
                
        
        
def generate_off_file_2d(name):
    os.remove(DIR+name+"_.txt") if os.path.isfile(DIR+name+"_.off") else None 
    os.remove(DIR+name+"__.txt") if os.path.isfile(DIR+name+"_.off") else None        
    os.remove(DIR+name+"_.off") if os.path.isfile(DIR+name+"_.off") else None
    os.remove(DIR+name+".off") if os.path.isfile(DIR+name+".off") else None
    
    cmd = "cat "+DIR+name+".txt | qhalf Qb2:0B2:0 Fp >>"+DIR+name+"_.txt"    
    try :
        subprocess.check_output(cmd,shell=True)
        print("qHalf 2D OK for file : "+name+".txt")
    except subprocess.CalledProcessError:
        print("Error in qHalf 2D for file "+name+".txt")
        
    # add arbitrary z value for each point (eg 0;2)
    with open(DIR+name+"_.txt") as inFile, open(DIR+name+"__.txt","w") as outFile:
        for i,line in enumerate(inFile):
            if i == 0:
                outFile.write("3\n")
            elif i == 1:
                num_pt = int(line)
                outFile.write(str(num_pt*2)+"\n")
            else:
                xy = line.rstrip("\n")
                outFile.write(xy+" "+str(0.0)+"\n")
                outFile.write(xy+" "+str(1.5)+"\n")
        inFile.close()
        outFile.close() 
    # call qconvex with new file
    cmd = "cat "+DIR+name+"__.txt | qconvex o >> "+DIR+name+"_.off"
    try :
        subprocess.check_output(cmd,shell=True)
        print("qHull OK for file : "+name+".txt")
    except subprocess.CalledProcessError:
        print("Error in qHull for file "+name+".txt")    
    #replace first line with "OFF"
    with open(DIR+name+"_.off") as inFile, open(DIR+name+".off","w") as outFile:
        for i,line in enumerate(inFile):
            if i == 0:
                outFile.write("OFF\n")
            else :
                outFile.write(line)
    inFile.close()
    outFile.close()   
    os.remove(DIR+name+"_.off")
    os.remove(DIR+name+"__.txt")
    os.remove(DIR+name+"_.txt")
    

def convert_off_dae(name):
    os.remove(DIR+name+"_.dae") if os.path.isfile(DIR+name+"_.dae") else None
    os.remove(DIR+name+".dae") if os.path.isfile(DIR+name+".dae") else None  

    cmd = "ctmconv "+DIR+name+".off "+DIR+name+"_.dae --upaxis Z --flip --calc-normals"
    try :
        subprocess.check_output(cmd,shell=True)
        print("convert constraints files to dae OK.")
    except subprocess.CalledProcessError:
        print("Error during conversion to dae ... ")
    # insert lines after <assert> : 
    with open(DIR+name+"_.dae") as inFile, open(DIR+name+".dae","w") as outFile:
        for line in inFile:
            if line.lstrip().startswith("<asset>"):
                outFile.write(line)
                outFile.write("    <unit name=\"meter\" meter=\"1\"/>\n")
                outFile.write("    <up_axis>Z_UP</up_axis>\n")
            else:
                outFile.write(line)
    inFile.close()
    outFile.close()
    os.remove(DIR+name+"_.dae")
    

def insert_color_material(name,materialName,color,alpha):
    os.remove(DIR+name+"_.dae") if os.path.isfile(DIR+name+"_.dae") else None
    if os.path.isfile(DIR+name+".dae"):
        os.rename(DIR+name+".dae",DIR+name+"_.dae")
    else:
        print("Error, file doesn't exist : "+DIR+name+".dae")
        
    # insert the declaration of the material : 
    with open(DIR+name+"_.dae") as inFile, open(DIR+name+".dae","w") as outFile:
        for line in inFile:        
            if line.lstrip().startswith("</asset>"):
                outFile.write(line)
                outFile.write("  <library_effects>\n")
                outFile.write("    <effect id=\""+materialName+"-effect\">\n")
                outFile.write("      <profile_COMMON>\n")
                outFile.write("        <technique sid=\"common\">\n")
                outFile.write("          <phong>\n")
                outFile.write("            <emission>\n")
                outFile.write("              <color sid=\"emission\">0 0 0 1</color>\n")
                outFile.write("            </emission>\n")
                outFile.write("            <ambient>\n")
                outFile.write("              <color sid=\"ambient\">0 0 0 1</color>\n")
                outFile.write("            </ambient>\n")
                outFile.write("            <diffuse>\n")
                outFile.write("              <color sid=\"diffuse\">"+str(color[0])+" "+str(color[1])+" "+str(color[2])+" "+str(alpha)+"</color>\n")
                outFile.write("            </diffuse>\n")
                outFile.write("            <specular>\n")
                outFile.write("              <color sid=\"specular\">"+str(color[0])+" "+str(color[1])+" "+str(color[2])+" "+str(alpha)+"</color>\n")
                outFile.write("            </specular>\n")
                outFile.write("            <shininess>\n")
                outFile.write("              <float sid=\"shininess\">50</float>\n")
                outFile.write("            </shininess>\n")
                outFile.write("            <transparency>\n")
                outFile.write("              <float sid=\"transparency\">"+str(1.)+"</float>\n")
                outFile.write("            </transparency>\n")
                outFile.write("            <index_of_refraction>\n")
                outFile.write("              <float sid=\"index_of_refraction\">1</float>\n")
                outFile.write("            </index_of_refraction>\n")
                outFile.write("          </phong>\n")
                outFile.write("        </technique>\n")
                outFile.write("      </profile_COMMON>\n")
                outFile.write("    </effect>\n")
                outFile.write("  </library_effects>\n")
                outFile.write("  <library_materials>\n")
                outFile.write("    <material id=\""+materialName+"-material\" name=\""+materialName+"\">\n")
                outFile.write("      <instance_effect url=\"#"+materialName+"-effect\"/>\n")
                outFile.write("    </material>\n")
                outFile.write("  </library_materials>\n")
            elif line.lstrip().startswith("<triangles"):
                num_triangles = line.split("\"")[1]
                outFile.write("                <triangles material=\""+materialName+"-material\" count=\""+num_triangles+"\">\n")
            elif line.lstrip().startswith("<instance_geometry url"):
                outFile.write(line.rstrip(" />\n")+">\n") # remove closing \
                outFile.write("          <bind_material>\n")    
                outFile.write("            <technique_common>\n")    
                outFile.write("              <instance_material symbol=\""+materialName+"-material\" target=\"#"+materialName+"-material\"/>\n")    
                outFile.write("            </technique_common>\n")    
                outFile.write("          </bind_material>\n")    
                outFile.write("        </instance_geometry>\n")    
            else:            
                outFile.write(line)    
        inFile.close()
        outFile.close()
        os.remove(DIR+name+"_.dae")
                                
                
    
global i_stab
i_stab=0
global i_kin
i_kin=0
global i_const
i_const = 0
global i_two_step
i_two_step=0
global i_bezier
i_bezier=0
    

def displayStabilityConstraints(r,quasiStatic=False):
    global i_stab
    if quasiStatic:
        generate_off_file_2d(STAB_NAME)
    else:
        generate_off_file(STAB_NAME)
        
    convert_off_dae(STAB_NAME)
    insert_color_material(STAB_NAME,"yellow",[1,0.75,0],0.5)
    
    r.client.gui.addMesh("stab_constraint_"+str(i_stab),DIR+STAB_NAME+".dae")
    r.client.gui.addToGroup("stab_constraint_"+str(i_stab),r.sceneName)        
    i_stab +=1

    
def displayKinematicsConstraints(r):
    global i_kin
    generate_off_file(KIN_NAME)
    convert_off_dae(KIN_NAME)
    insert_color_material(KIN_NAME,"green",[0,1,0],0.3)
    
    r.client.gui.addMesh("kin_constraint_"+str(i_kin),DIR+KIN_NAME+".dae")
    r.client.gui.addToGroup("kin_constraint_"+str(i_kin),r.sceneName)        
    i_kin +=1    
    
def displayALlConstraints(r):
    global i_const
    generate_off_file(CONS_NAME)
    convert_off_dae(CONS_NAME)
    insert_color_material(CONS_NAME,"red",[1,0,0],0.7)

    r.client.gui.addMesh("all_constraint_"+str(i_const),DIR+CONS_NAME+".dae")
    r.client.gui.addToGroup("all_constraint_"+str(i_const),r.sceneName)        
    i_const +=1    
    

def displayOneStepConstraints(r,quasiStatic=False):    
    removeAllConstraints(r)
    displayKinematicsConstraints(r)
    displayStabilityConstraints(r,quasiStatic)        
    displayALlConstraints(r)
    
    
def displayTwoStepConstraints(r,inter_exist):
    removeAllConstraints(r)
    global i_two_step
    generate_off_file(CONS_NAME+"_break")
    convert_off_dae(CONS_NAME+"_break")
    insert_color_material(CONS_NAME+"_break","blue",[0,0,1],0.5)
    generate_off_file(CONS_NAME+"_create")
    convert_off_dae(CONS_NAME+"_create")
    insert_color_material(CONS_NAME+"_create","blue",[0,0,1],0.5) 
    if inter_exist:
        displayALlConstraints(r)

    r.client.gui.addMesh("constraint_twoStep_b"+str(i_two_step),DIR+CONS_NAME+"_break"+".dae")
    r.client.gui.addToGroup("constraint_twoStep_b"+str(i_two_step),r.sceneName)            
    r.client.gui.addMesh("constraint_twoStep_c"+str(i_two_step),DIR+CONS_NAME+"_create"+".dae")
    r.client.gui.addToGroup("constraint_twoStep_c"+str(i_two_step),r.sceneName)      
    i_two_step +=1    
    
    
def removeAllConstraints(r):
    global i_stab
    global i_kin
    global i_const
    global i_two_step
    global i_bezier

    r.client.gui.removeFromGroup("constraint_twoStep_c"+str(i_two_step-1),r.sceneName)
    r.client.gui.removeFromGroup("constraint_twoStep_b"+str(i_two_step-1),r.sceneName)
    r.client.gui.removeFromGroup("all_constraint_"+str(i_const-1),r.sceneName)
    r.client.gui.removeFromGroup("kin_constraint_"+str(i_kin-1),r.sceneName)
    r.client.gui.removeFromGroup("stab_constraint_"+str(i_stab-1),r.sceneName)
    r.client.gui.removeFromGroup("bezier_constraint_"+str(i_bezier-1),r.sceneName)
    


def displayBezierConstraints(r):
    removeAllConstraints(r)
    global i_bezier
    generate_off_file(BEZIER_NAME)
    #generate_off_file_2d(BEZIER_NAME)
    convert_off_dae(BEZIER_NAME)
    insert_color_material(BEZIER_NAME,"green",[0,1,0],0.3)
    
    r.client.gui.addMesh("bezier_constraint_"+str(i_bezier),DIR+BEZIER_NAME+".dae")
    r.client.gui.addToGroup("bezier_constraint_"+str(i_bezier),r.sceneName)        
    i_bezier +=1        
