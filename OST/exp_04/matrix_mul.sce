clc;
clear;

A = [1 2 3;
     4 5 6;
     7 8 10];

B = [2 1 3;
     1 0 2;
     4 1 5];

// Display Matrix A
disp("Elements of Matrix A:");
disp(A);

// Display Matrix B
disp("Elements of Matrix B:");
disp(B);

// Addition
C = A + B;
disp("Addition A + B:");
disp(C);

// Subtraction
D = A - B;
disp("Subtraction A - B:");
disp(D);

// Matrix Multiplication
E = A * B;
disp("Matrix Multiplication A * B:");
disp(E);

// Element-wise Multiplication
F = A .* B;
disp("Element-wise Multiplication A .* B:");
disp(F);

// Transpose
T = A';
disp("Transpose of A:");
disp(T);

// Determinant
detA = det(A);
disp("Determinant of A:");
disp(detA);

// Inverse
invA = inv(A);
disp("Inverse of A:");
disp(invA);

// Matrix Size
s = size(A);
disp("Size of Matrix A:");
disp(s);

// Identity Matrix
I = eye(3,3);
disp("Identity Matrix:");
disp(I);

// Zero Matrix
Z = zeros(3,3);
disp("Zero Matrix:");
disp(Z);

// Ones Matrix
O = ones(3,3);
disp("Ones Matrix:");
disp(O);
