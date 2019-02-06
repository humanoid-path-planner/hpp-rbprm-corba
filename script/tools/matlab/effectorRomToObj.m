function [] = effectorRomToObj(filename, outname)
delimiterIn = ',';
headerlinesIn = 0;
points = importdata(filename,delimiterIn,headerlinesIn);
k = convhull(points);
vertface2obj(points,k,outname)
