clc;
clear;
clf;

x = 0:0.01:2*%pi;

y1 = sin(x);
y2 = cos(x);

plot(x,y1);
plot(x,y2);
xtitle("Sine and Cosine Functions", "x", "y");

legend("sin(x)", "cos(x)");

xgrid();
