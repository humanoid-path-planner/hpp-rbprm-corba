from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np

f = open("/home/pfernbac/Documents/com_ineq_test/timings_hyq_flat_v02.txt","r")
results={}
quasiStaticResults={}
OnlyDynamicResults={}
dicP0 = {}
dicP1={}
dicP2={}
dicPhases = [dicP0,dicP1,dicP2]


for line in f.readlines():
    tab = line.rstrip('\n').split(" ")
    timings =[]
    for s in tab[:3]:
        timings +=[float(s)]
    success = (int(tab[3])==1)
    quasiStatic = (int(tab[4])==1)
    if success:
        if (timings[0],timings[1],timings[2]) in results :
            # there is already a success for this timing, increase the number
            results[(timings[0],timings[1],timings[2])] += 1
        else :
            #create an entry in the dic : 
            results[(timings[0],timings[1],timings[2])] = 1
        for i in range(3):
            if timings[i] in dicPhases[i]:
                dicPhases[i][timings[i]] += 1
            else :
                dicPhases[i][timings[i]] = 1
        if quasiStatic:
            if (timings[0],timings[1],timings[2]) in quasiStaticResults :
                # there is already a success for this timing, increase the number
                quasiStaticResults[(timings[0],timings[1],timings[2])] += 1
            else :
                #create an entry in the dic : 
                quasiStaticResults[(timings[0],timings[1],timings[2])] = 1   
        else :
            if (timings[0],timings[1],timings[2]) in OnlyDynamicResults :
                # there is already a success for this timing, increase the number
                OnlyDynamicResults[(timings[0],timings[1],timings[2])] += 1
            else :
                #create an entry in the dic : 
                OnlyDynamicResults[(timings[0],timings[1],timings[2])] = 1  
                
                
f.close()


#compute arrays from dic :
x = []
y = []
z = []
v = []
for keys in results.keys():
    x += [keys[0]]
    y += [keys[1]]
    z += [keys[2]]
    v += [results[keys]]

# plot a scatter 3D 
max_value = max(results.values())
cmhot = plt.get_cmap("hot")
range_value = range(max_value)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.scatter(x,y,z,v,c=v,cmap=cmhot)

ax.set_xlabel("timing P1 (s)")
ax.set_ylabel("timing P2 (s)")
ax.set_zlabel("timing P3 (s)")
ax.set_title("success for each timings : yellow = more success, red = less success")

# plot histogram for each phase (each timings taken independantly)

t_range = dicP0.keys()

for i in range(3):
    values = []    
    for t in t_range:
        if t in dicPhases[i]:
            values += [dicPhases[i][t]]
        else:
            values += [0]
        
    fig = plt.figure()
    plt.bar(t_range,values,0.04)
    plt.title("success for timing of phase "+str(i))
    

plt.show()

