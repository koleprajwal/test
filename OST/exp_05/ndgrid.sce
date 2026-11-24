clc;
clear;
clf;

x = -5:0.2:5;
y = -5:0.2:5;

[X,Y] = ndgrid(x,y);

Z = X.^2 + Y.^2;

plot3d(x, y, Z);

xtitle("3D Surface Plot", "X", "Y", "Z");