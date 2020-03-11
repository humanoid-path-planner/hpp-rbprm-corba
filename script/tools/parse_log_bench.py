import sys


num_transition = 0.
valid_transition = 0.
traj_length = 0.
valid_length = 0.
run = 0.
f = open("/local/fernbac/bench_iros18/kin_constraint_tog/without_kin_constraints.log","r")
for line in f.readlines():
    t = line.rstrip("\n").split(" ")
    i = 1
    while len(t[i]) == 0:
        i +=1
    if str(t[0]).startswith("num_transition") :
        run += 1
        num_transition += float(t[i].rstrip(' '))
    if str(t[0]) == "valid_transition" :
        valid_transition += float(t[i].rstrip(' '))
    if str(t[0]).startswith("traj_length") :
        traj_length += float(t[i].rstrip(' '))           
    if str(t[0]).startswith("valid_length") :
        valid_length += float(t[i].rstrip(' '))

f.close()
print("for "+str(run)+" runs : ")
print("total num transition              : "+str(int(num_transition)))
print("total valid transition            : "+str(int(valid_transition)))
print("percentage of feasible transition : "+str(float(valid_transition/num_transition)*100.)+" %")
print("total length of trajectory tested : "+str(traj_length))
print("total length of valid trajectory  : "+str(valid_length))
print("percentage of feasible trajectory : "+str(float(valid_length/traj_length)*100.)+" %")
