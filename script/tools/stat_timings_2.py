import plotly.plotly as py
import plotly.graph_objs as go
import plotly
import numpy as np

#~ f = open("./timings_hyq_flat_v02.txt","r")
#~ f = open("./timings_hyq_planches_v02.txt","r")

resultsOrdered = {}
total_states = 0

filterMin = 0.05


def addData(filename):    
    f = open(filename,"r")
    global resultsOrdered
    global total_states
    with open(filename,"r") as infile:
        for line in infile:
            if line.startswith("#"):
                total_states += 1
            else:
                tab = line.rstrip('\n').split(" ")
                timings =[]
                for s in tab[:3]:
                    timings +=[float(s)]
                success = (int(tab[3])==1)
                quasiStatic = (int(tab[4])==1)
                v = 1 if success else 0
                if timings[1] >= filterMin:
                    if (timings[0],timings[1],timings[2]) in resultsOrdered :
                        resultsOrdered[(timings[0],timings[1],timings[2])] += [str(v)]        
                    else :
                        resultsOrdered[(timings[0],timings[1],timings[2])] = [str(v)]            
                            
    print("file : "+filename+" parsed successfully. Current number of states : "+str(total_states))
    f.close()
    

timings_fused = {}
def fuseData():
    # now fuse all numbers with equal success results
    global timings_fused
    for timing, ordSucc in resultsOrdered.items():
        ordSucc = map(str, ordSucc)
        idxOrd = ''.join(ordSucc)
        if idxOrd in timings_fused : 
            timings_fused[idxOrd] += [timing]
        else:
            timings_fused[idxOrd]  = [timing]
            
    print("num different values", len(timings_fused))
    return timings_fused

totalSucc = 0
sortedIds = None

def sort_by_best_timings():
    global totalSucc
    global sortedIds
    def num_1_com(x, y):
        return y.count('1') - x.count('1')
        
    sortedIds = sorted(timings_fused.keys(), cmp=num_1_com)

    from functools import reduce
    totalSucc = float(reduce((lambda x, y: '{0:b}'.format(int(x, base=2) or int(y, base=2))), sortedIds).count('1'))

def percentageCovered(inputString):
    return float(inputString.count('1')) / totalSucc
    
def computeBestMatch(currentString, iterValues, aggTimings):
    intCurrent = int(currentString, base=2)
    res = [('{0:b}'.format(intCurrent | int(el, base=2))).count('1') for el in iterValues]
    idMax = res.index(max(res))
    bestString = iterValues[idMax]
    del iterValues[idMax]
    return '{0:b}'.format(intCurrent | int(bestString, base=2)), bestString
    
def getNumSamplesToPercentageTarget(target = 0.95):
    currentString = sortedIds[0]
    iterValues = sortedIds[1:]
    aggTimings = [0]
    while percentageCovered(currentString) < target:
        newCurrent, bestString = computeBestMatch(currentString, iterValues, aggTimings)
        if(newCurrent != currentString):
            aggTimings += [sortedIds.index(bestString)]
            currentString = newCurrent
    return aggTimings

def computeTimingsForPercentage(targetPercentage = 0.35):
    aggTimings = getNumSamplesToPercentageTarget(target = targetPercentage)
    print("total number of timings required to cover {0} % of the successes :".format(targetPercentage * 100.), len(aggTimings))
    return aggTimings
    


filenames = ["./hyq/flat/timings_logs_v0.log", "./hyq/planches/timings_hyq_planches_v02.log","./hyq/planches/timings_hyq_planches_vnc.log","./hrp2/flat/timings_logs_v02.log"]
requiredPercentages = [1.,0.95,0.9,0.2]
aggTimings = []

for filename in filenames:
    addData(filename)

fuseData()
sort_by_best_timings()

for perc in requiredPercentages:
    aggTimings  += [computeTimingsForPercentage(perc)]
    
def getTimingsFromAggIndex(aggIndex):
    return timings_fused[sortedIds[aggIndex]]
    
results = getTimingsFromAggIndex(aggTimings[-1][-1])
results


import pickle as pickle

#pickle.dump( [resultsOrdered,timings_fused], open( "timings_hyq_hrp2.p", "wb" ) )