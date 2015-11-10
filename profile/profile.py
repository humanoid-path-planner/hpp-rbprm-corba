#!/usr/bin/env python

import subprocess as sp
import os

scenarios = ['stair_bauzil_hrp2']#,'standing_hrp2','darpa_hrp2']
n_trials = 3


python_script_extension = "_interp.py"
analysis = {}
lower = -100000
higher = 100000000

#~ 
#~ *** PROFILING RESULTS [ms] (min - avg - max - lastTime - nSamples) ***
#~ complete generation                     5110.09	5110.09	5110.09	5110.09	1
#~ test balance                            0.09	0.22	0.60	0.54	1199
#~ contact: 20
#~ no contact: 8
#~ planner succeeded: 1
#~ unstable contact: 2
#~ path computation 0.81
#~ 
#~ *** PROFILING RESULTS [ms] (min - avg - max - lastTime - nSamples) ***
#~ complete generation                     7162.32	7162.32	7162.32	7162.32	1
#~ test balance                            0.09	0.20	0.63	0.50	1431
#~ contact: 21
#~ no contact: 11
#~ planner failed: 1
#~ unstable contact: 3

def avg(l):
	return reduce(lambda x, y: x + y, l) / len(l)


def parseData(scenario):
	filename = scenario+"_log.txt";
	#~ os.rename("log.txt", filename)
	analysis[scenario] = {}
	data = analysis[scenario]
	data['no contact'] = 0
	data['contact'] = 0	
	data['unstable contact'] = 0	
	data['balance_min'] = higher
	data['balance_max'] = lower
	data['balance'] = []
	data['minnumbalance'] = higher
	data['maxnumbalance'] = lower
	data['numbalance'] = []
	data['mingen_time'] = higher
	data['maxgen_time'] = lower
	data['gen_time'] = []
	data['failed'] = 0
	data['success'] = 0
	data['path_time'] = []
	file = open(filename,"r+");
	for line in file.readlines():
		if not (line.find('complete generation') == -1):
			time = float(line.split()[3])
			data['mingen_time'] = min(data['mingen_time'],float(time))
			data['maxgen_time'] = max(data['maxgen_time'],float(time))
			data['gen_time'].append(time);
		elif not (line.find('test balance') == -1):
			print line.split()
			_1, _2, _min, _avg, _max, _last, _numiter = line.split()
			data['balance_min'] = min(data['balance_min'],float(_min))
			data['balance_max'] = max(data['balance_max'],float(_max))
			data['minnumbalance'] = min(data['minnumbalance'],float(_numiter))
			data['maxnumbalance'] = max(data['maxnumbalance'],float(_numiter))		
			data['numbalance'].append(float(_numiter))
			for i in range(0,1): #int(_numiter)):				
				data['balance'].append(float(_avg))
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
			data['path_time'].append(float(line.rstrip("\n").split()[2]));

def analyzeData():
	for scenario in scenarios:
		data = analysis[scenario]
		data['balance'] = avg(data['balance'])
		data['numbalance'] = avg(data['numbalance'])
		data['path_time'] = avg(data['path_time'])
		data['gen_time'] = avg(data['gen_time'])
	print(analysis)
	
for scenario in scenarios:
	for i in range(0,n_trials):
		var = scenario+python_script_extension;
		#~ print(var);
		#~ sp.check_call(["./profile.sh", str(var)]);
	# move log
	parseData(scenario)
	analyzeData()
	#~ os.rename("logpath.txt", scenario+"_log_path.txt")
