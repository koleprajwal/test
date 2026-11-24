clc;
clear;
clf;

x = -5:0.2:5;
y = -5:0.2:5;

z = zeros(length(x), length(y));

for i = 1:length(x)
    for j = 1:length(y)
        z(i,j) = x(i)^2 + y(j)^2;
    end
end

plot3d(x, y, z);

xtitle("3D Surface: z = x^2 + y^2", "X", "Y", "Z");