import sys

#save the total time of the planning if provided : 

def parseBenchmark(tPlanningTab):

  if len(tPlanningTab)>1 :
    tPlanning = 3600*tPlanningTab[0]+tPlanningTab[1]*60+tPlanningTab[2]+tPlanningTab[3]/1000.
  else:
    tPlanning = 0



  # format : [NAME, initial time value, initial numberOfIterations]
  dic = []
  dic.append(["ONE_STEP:",0.,0])
  dic.append(["SHOOT:",0.,0])
  dic.append(["NEAREST:",0.,0])
  dic.append(["EXTEND:",0.,0])
  dic.append(["steering_kino:",0.,0])
  dic.append(["FILL_NODE_MATRICE:",0.,0])
  dic.append(["COMPUTE_INTERSECTION:",0.,0])
  dic.append(["COMPUTE_DIRECTION:",0.,0])
  dic.append(["FIND_A_MAX:",0.,0])
  dic.append(["INTERMEDIATE_ACCELERATION_CHECKS:",0.,0])
  dic.append(["PATH_VALIDATION:",0.,0])
  dic.append(["DYNAMIC_VALIDATION:",0.,0])
  dic.append(["ORIENTED_OPTIMIZER:",0.,0])
  dic.append(["RANDOM_SHORTCUT:",0.,0])
  dic.append(["SHOOT_COLLISION:",0.,0])
  dic.append(["IS_REACHABLE:",0.,0])
  dic.append(["QP_REACHABLE:",0.,0])
  dic.append(["REACHABLE_STABILITY:",0.,0])
  dic.append(["REACHABLE_KINEMATIC:",0.,0])
  dic.append(["REACHABLE_STACK:",0.,0])
  dic.append(["REACHABLE_CALL_CENTROIDAL:",0.,0])
  dic.append(["IS_REACHABLE_DYNAMIC:",0.,0])
  dic.append(["SOLVE_TRANSITION_ONE_STEP:",0.,0])

  import subprocess
  process = subprocess.Popen("pidof hpp-rbprm-server".split(),stdout=subprocess.PIPE)
  output, error = process.communicate()
  output = output.rstrip('\n')

  f = open("/local/dev_hpp/logs/benchmark."+output+".log","r")


  for line in f.readlines():
    t = line.split(" ")
    if len(t)>=3:
      if str(t[1]).startswith("Time_Counter"):
        varName = t[2]
        sit = str(t[3]).rstrip(",")
        it = float(sit)
        var = str(t[4]).rstrip(",")
      else:
        it=1
        varName = t[1]
        var = str(t[2])
      if(var.startswith("0")): ## take minutes and hours in account
        ts=var.split(".")
        stime = str(ts[-1])
        stime = stime.rstrip('\n')
        time=float(stime)/1000.
        for i in range(0,len(dic)):
          if varName==dic[i][0]:
            dic[i][1] = dic[i][1]+time
            dic[i][2] = dic[i][2]+it


  print("Benchmark results (average, in ms): \n")


  for i in range(0,len(dic)):
    if(dic[i][2] > 0):
      t = dic[i][1] / dic[i][2]
      print(dic[i][0],t)
      print(str(dic[i][2])+" iterations")

  if tPlanning > 0:
    print("\nDecomposition of the total planning time of "+str(tPlanning)+"s:")
    mt = tPlanning*1000.
    print("# One step : "+str((dic[0][1]/mt)*100.)+" %")
    print("\t * Random shooting : "+str((dic[1][1]/mt)*100.)+" %")
    print("\t \t - collision checks : "+str((dic[14][1]/mt)*100.)+" %")
    print("\t * Nearest search  : "+str((dic[2][1]/mt)*100.)+" %")
    print("\t * extend          : "+str((dic[3][1]/mt)*100.)+" %")
    print("\t \t - steering kino (core) : "+str((dic[4][1]/mt)*100.)+" %")
    print("\t \t - fill node matrice    : "+str((dic[5][1]/mt)*100.)+" %")
    print("\t \t \t . compute intersection (fcl) : "+str((dic[6][1]/mt)*100.)+" %")
    print("\t \t - compute a max        : "+str((dic[8][1]/mt)*100.)+" %")
    print("\t \t - check accelerations  : "+str((dic[9][1]/mt)*100.)+" %")
    print("\t * path validation : "+str((dic[10][1]/mt)*100.)+" %")
    print("\t \t - Dynamic validation : "+str((dic[11][1]/mt)*100.)+" %")
    print("# Random shortcut       : "+str((dic[13][1]/mt)*100.)+" %")
    print("# Orientation optimizer : "+str((dic[12][1]/mt)*100.)+" %")


    print("% Is Reachable test     : called "+str(dic[15][2])+" times, average of "+str(dic[15][1]/dic[15][2])+" ms.")
    print("% QP solver isReachable : called "+str(dic[16][2])+" times, average of "+str(dic[16][1]/dic[16][2])+" ms.") 
    print("% stability constraints : called "+str(dic[17][2])+" times, average of "+str(dic[17][1]/dic[17][2])+" ms.") 
    print("% kinematic constraints : called "+str(dic[18][2])+" times, average of "+str(dic[18][1]/dic[18][2])+" ms.") 
    print("% stack matrices        : called "+str(dic[19][2])+" times, average of "+str(dic[19][1]/dic[19][2])+" ms.") 
    print("# Is Reachable Dynamic  : called "+str(dic[20][2])+" times, average of "+str(dic[20][1]/dic[20][2])+" ms.") 
    print("# Qp transition         : called "+str(dic[21][2])+" times, average of "+str(dic[21][1]/dic[21][2])+" ms.") 



