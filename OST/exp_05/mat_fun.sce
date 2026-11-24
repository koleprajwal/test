clc;
clear;
clf;

x = -5:0.1:5;

y1 = x;
y2 = x.^2;
y3 = x.^3;

plot(x, y1);
plot(x, y2);
plot(x, y3);

xtitle("Multiple Mathematical Functions", "x", "y");
legend("y = x", "y = x^2", "y = x^3");
xgrid();



