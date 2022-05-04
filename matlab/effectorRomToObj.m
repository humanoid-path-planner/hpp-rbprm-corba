% Open it on blender with default axis setting and export it with x forward.%
    Use decimate to reduce the number of faces,
    don't forget to triangulate faces before exporting

    function[] = effectorRomToObj(filename, outname) delimiterIn = ',';
headerlinesIn = 0;
points = importdata(filename, delimiterIn, headerlinesIn);
k = convhull(points);
vertface2obj(points, k, outname)
