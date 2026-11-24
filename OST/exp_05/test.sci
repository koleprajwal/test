clc;
clear;
clf;

x = -%pi:0.1:%pi;
y = -%pi:0.1:%pi;

[X,Y] = ndgrid(x,y);

Z = sin(X).*cos(Y);

plot3d(x, y, Z);

xtitle("3D Surface: z = sin(x)cos(y)", "X", "Y", "Z");
