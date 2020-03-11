import plotly.plotly as py
import plotly.graph_objs as go
import plotly
import numpy as np

f = open("/home/pfernbac/Documents/com_ineq_test/timings_hyq_planches_v02.txt","r")
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
    v = 1 if success else 0
    if (timings[0],timings[1],timings[2]) in results :
        # there is already a success for this timing, increase the number
        results[(timings[0],timings[1],timings[2])] += v
    else :
        #create an entry in the dic : 
        results[(timings[0],timings[1],timings[2])] = v
    for i in range(3):
        if timings[i] in dicPhases[i]:
            dicPhases[i][timings[i]] += v
        else :
            dicPhases[i][timings[i]] = v
    if success :
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
p0 = []
p1 = []
p2 = []
v = []
for keys in results.keys():
    p0 += [keys[0]]
    p1 += [keys[1]]
    p2 += [keys[2]]
    v += [results[keys]]

# plot a scatter 3D 

trace1 = go.Scatter3d(
    x=p0,
    y=p2,
    z=p1,
    mode='markers',
    marker=dict(
        size=5,
        color=v,                # set color to an array/list of desired values
        colorscale='Hot',   # choose a colorscale
        opacity=0.8
    )
)

data = [trace1]
layout = go.Layout(
    title='Success on flat ground : more yellow = more success',    
    scene = dict(
    xaxis=dict(
        title='timing p0 (4 supports) (s)',
        titlefont=dict(
            family='Courier New, monospace',
            size=12,
            color='#7f7f7f'
        )
    ),
    yaxis=dict(
        title='timing p2 (4 supports) (s)',
        titlefont=dict(
            family='Courier New, monospace',
            size=12,
            color='#7f7f7f'
        )
    ),
    zaxis=dict(
        title='timing p1 (3 supports) (s)',
        titlefont=dict(
            family='Courier New, monospace',
            size=12,
            color='#7f7f7f'
        )
    )
    )
)
fig = go.Figure(data=data, layout=layout)
plotly.offline.plot(fig,filename="/home/pfernbac/Documents/com_ineq_test/3D-hyq")


# plot histogram for each phase (each timings taken independantly)
"""
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
"""

# fix tuple with the maximal value : 
m = max(results.values())
for keys in results:
    if results[keys] == m:
        print(keys)
    