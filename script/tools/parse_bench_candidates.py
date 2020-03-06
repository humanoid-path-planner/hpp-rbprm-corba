import sys

current_candidates = 0.
total_candidates = 0.
total_success = 0.
f = open("/local/fernbac/bench_iros18/bench_contactGeneration/candidatesWalkNoCroc.log","r")

for line in f.readlines():
    if line.startswith("BeginInterpolation"):
        current_candidates = 0.
    if line.startswith("evaluatedCandidates"):
        t = line.rstrip("\n").split(" ")
        current_candidates += int(t[1])
    if line.startswith("succeed"):
        total_success += 1.
        total_candidates += current_candidates
        current_candidates = 0.
    
    

print("for "+str(total_success)+" success :")
print("average : "+str((total_candidates/total_success))+" candidate per runs.")
   
