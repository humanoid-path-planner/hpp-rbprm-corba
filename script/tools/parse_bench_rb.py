import sys


dic = []
dic.append(["total_path_computed",0.,0])
dic.append(["total_path_validated",0.,0])
dic.append(["percentage_valide_direction",0.,0])
dic.append(["num_nodes",0.,0])



f = open("/local/dev_hpp/benchs/benchHro_slope_easy.txt","r")


for line in f.readlines():
  t = line.split(" ")
  if len(t)>=3:
    varName = t[0]
    var = float(t[2])
    for i in range(0,len(dic)):
      if varName==dic[i][0]:
        dic[i][1] = dic[i][1]+var
        dic[i][2] = dic[i][2]+1







for i in range(0,len(dic)):
  if(dic[i][2] > 0):
    t = dic[i][1] / dic[i][2]
    print(dic[i][0],t)




