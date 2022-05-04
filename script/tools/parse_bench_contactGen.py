import sys

totalSuccess = 0.0
totalTime = 0.0
totalMuscodConverg = 0.0
totalMuscodWarmStartConverg = 0.0
totalCrocConverg = 0.0
totalConf = 0.0
totalIt = 0.0

f = open("/local/fernbac/bench_iros18/bench_contactGeneration/walk_noCroc.log", "r")
line = f.readline()

while line.startswith("new"):
    totalIt += 1.0
    line = f.readline()
    t = line.rstrip("\n").split(" ")
    assert t[0].startswith("success")
    success = t[1].startswith("True")
    if success:
        totalSuccess += 1.0
    line = f.readline()
    t = line.rstrip("\n").split(" ")
    assert t[0].startswith("muscodNoWarmStart")
    if t[1].startswith("True"):
        totalMuscodConverg += 1.0
    line = f.readline()
    t = line.rstrip("\n").split(" ")
    if t[0].startswith("crocConverged"):
        if t[1].startswith("True"):
            totalCrocConverg += 1.0
    elif t[0].startswith("muscodWarmStart"):
        if t[1].startswith("True"):
            totalMuscodWarmStartConverg += 1.0
    else:
        print("badly formated log")
    line = f.readline()
    t = line.rstrip("\n").split(" ")
    assert t[0].startswith("time")
    if success:
        totalTime += float(t[1])
    line = f.readline()
    t = line.rstrip("\n").split(" ")
    assert t[0].startswith("configs")
    if success:
        totalConf += int(t[1])
    line = f.readline()


print("For : " + str(totalIt) + " runs : ")
print("success contact generation : " + str((totalSuccess / totalIt) * 100.0) + "  %")
print(
    "success muscod             : "
    + str((totalMuscodConverg / totalSuccess) * 100.0)
    + "  %"
)
print(
    "success muscod Warm Start  : "
    + str((totalMuscodWarmStartConverg / totalSuccess) * 100.0)
    + "  %"
)
print(
    "success croc converged :   : "
    + str((totalCrocConverg / totalSuccess) * 100.0)
    + "  %"
)
print("avg time                   : " + str(totalTime / totalSuccess) + "s")
print("avg configs                : " + str(totalConf / totalSuccess))
