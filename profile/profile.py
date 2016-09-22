#!/usr/bin/env python

import subprocess as sp
import os
import datetime

#~ scenarios = ['standing_hrp2']
#~ scenarios = ['car_hrp2']
#~ scenarios = ['stair_bauzil_hrp2']
scenarios = ['darpa_hyq', 'stair_bauzil_hrp2']

n_trials = 100 

stats = ['balance','collision','ik']

python_script_extension = "_interp.py"
analysis = {}
lower = -100000
higher = 100000000

def avg(l):
	if(len(l) == 0):
		return -1
	return reduce(lambda x, y: x + y, l) / len(l)


def initdata(name, data):
	n = name
	data[n+'_min'] = higher
	data[n+'_max'] = lower
	data[n] = []
	data['total_'+n+'_min'] = higher
	data['total_'+n+'_max'] = lower
	data['total_'+n] = []
	data['minnum'+n] = higher
	data['maxnum'+n] = lower
	data['num'+n] = []

def parseLine(sep, name, line, data):
	if sep == 1:
		_1, _min, _avg, _max, _totaltime, _numiter = line.split()
	elif sep ==2:
		_1, _2, _min, _avg, _max, _totaltime, _numiter = line.split()
	data[name+'_min'] = min(data[name+'_min'],float(_min))
	data[name+'_max'] = max(data[name+'_max'],float(_max))
	data['minnum'+name] = min(data['minnum'+name],float(_numiter))
	data['maxnum'+name] = max(data['maxnum'+name],float(_numiter))		
	data['num'+name].append(float(_numiter))
	for i in range(0,int(_numiter)):				
		data[name].append(float(_avg))
	total_time = float(_totaltime)
	data['total_'+name+'_min'] = min(data['total_'+name+'_min'],total_time)
	data['total_'+name+'_max'] = max(data['total_'+name+'_max'],total_time)
	data['total_'+name].append(total_time)
	
def parseData(scenario):
	filename = scenario+"_log.txt";
	os.rename("log.txt", filename)
	analysis[scenario] = {}
	data = analysis[scenario]
	data['no contact'] = 0
	data['contact'] = 0	
	data['unstable contact'] = 0	
	data['mingen_time'] = higher
	data['maxgen_time'] = lower
	data['gen_time'] = []
	data['failed'] = 0
	data['success'] = 0
	data['path_time'] = []
	data['min_path_time'] = higher
	data['max_path_time'] = lower
	for stat in stats:
		initdata(stat, data)	
	file = open(filename,"r+");
	for line in file.readlines():
		if not (line.find('complete generation') == -1):
			time = float(line.split()[3])
			data['mingen_time'] = min(data['mingen_time'],float(time))
			data['maxgen_time'] = max(data['maxgen_time'],float(time))
			data['gen_time'].append(time);
		elif not (line.find('test balance') == -1):
			parseLine(2, 'balance', line, data)
		elif not (line.find('collision') == -1):
			parseLine(1, 'collision', line, data)
		elif not (line.find('ik') == -1):
			parseLine(1, 'ik', line, data)
		elif not (line.find('no contact:') == -1):
			data['no contact'] = data['no contact'] + float(line.rstrip("\n").split()[2]);
		elif not (line.find('unstable contact:') == -1):
			data['unstable contact'] = data['unstable contact'] + float(line.rstrip("\n").split()[2]);			
		elif not (line.find('contact:') == -1):
			data['contact'] = data['contact'] + float(line.rstrip("\n").split()[1]);				
		elif not (line.find('planner failed:') == -1):
			data['failed'] = data['failed'] + 1;				
		elif not (line.find('planner succeeded') == -1):
			data['success'] = data['success'] + 1;		
		elif not (line.find('path computation') == -1):
			time = float(line.rstrip("\n").split()[2])
			data['path_time'].append(time);
			data['min_path_time'] = min(data['min_path_time'],time)
			data['max_path_time'] = max(data['max_path_time'],time)


def printOneStat(f, name, data, g_time, tTime):
	n = name
	d = data
	print(n, len(d[n]))
	d[n] = avg(d[n])
	d['num'+n] = avg(d['num'+n])
	d['total_'+n] = avg(d['total_'+n])
	f.write (n +" tests: \n")
	f.write ("\t single operation time (min / avg / max):\n \t " + str(d[n+'_min']) +  "\t" + str(d[n]) + "\t" + str(d[n+'_max']) + "\t \n")
	f.write ("\t total time (min / avg / max / % of total / % of total wtht path planning):\n \t " + str(d['total_'+n+'_min']) +  "\t" + str(d['total_'+n]) + "\t" + str(d['total_'+n+'_max']) + "\t" +  str(float(d['total_'+n]) / tTime * 100) + "%" + "\t" +  str(float(d['total_'+n]) / g_time * 100) + "%\n")
	f.write ("\t number of tests (min / avg / max):\n \t " + str(d['minnum'+n]) +  "\t" + str(d['num'+n]) + "\t" + str(d['maxnum'+n]) + "\n")
		
		
def analyzeData():
	f = open("log_"+str(datetime.datetime.now())+".txt","w+")
		
	for scenario in scenarios:
		d = analysis[scenario]
		d['path_time'] = avg(d['path_time'])
		d['gen_time'] = avg(d['gen_time'])
		g_time = d['gen_time']
		tTime = d['gen_time'] + d['path_time']
		f.write (scenario +":\n")
		
		f.write ("total computation time (min / avg / max): " + str(d['mingen_time'] + d['min_path_time']) + "\t" + str(tTime) + "\t" + str(d['maxgen_time'] + d['max_path_time']) + "\n")
		
		f.write ("generation time (without RRT) (min / avg / max / % of total):\n \t " + str(d['mingen_time']) + "\t" + str(d['gen_time']) + "\t" + str(d['maxgen_time']) + "\t"+ str(float(d['gen_time']) / tTime * 100) + "%\n")
		
		f.write ("path computation: \n")
		f.write ("\t time (min / avg / max / % of total):\n \t " + str(d['min_path_time']) +  "\t" + str(d['path_time']) + "\t" + str(d['max_path_time']) + "\t" +  str(float(d['path_time']) / tTime * 100) + "%\n")
		
		for stat in stats:
			printOneStat(f, stat, d, g_time, tTime)
			
		f.write ("number of successes / failures / ratio: " + str(d['success']) + "\t" + str(d['failed']) + "\t" + str(float(d['success']) / float(d['success']) * 100)  + "%\n")
		
		nc = float(d['no contact'])
		c = float(d['contact'])
		uc = float(d['unstable contact'])
		tc = nc + c + uc 
		f.write ("% of failed contact generation (no candidates found): " + str(nc / tc * 100)  + "%\n")
		f.write ("% of unstable contact generation (no balanced candidates found): " + str(uc / tc * 100)  + "%\n")
		f.write ("\n \n \n")
	f.close()
	
for scenario in scenarios:
	for i in range(0,n_trials):
		var = scenario+python_script_extension;
		sp.check_call(["./profile.sh", str(var)]);
	parseData(scenario)
analyzeData()
