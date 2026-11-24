clc;
clear;

// Creating Matrix A
A = [2 3 1;
     4 1 5;
     6 2 3];

// Creating Matrix B
B = [1 4 2;
     3 2 1;
     5 0 3];


disp("========== MATRIX A ==========");
disp(A);

disp("========== MATRIX B ==========");
disp(B);

// Addition
C = A + B;
disp("========== ADDITION (A + B) ==========");
disp(C);


// Subtraction
D = A - B;
disp("========== SUBTRACTION (A - B) ==========");
disp(D);


// Matrix Multiplication
E = A * B;
disp("========== MATRIX MULTIPLICATION (A * B) ==========");
disp(E);

// Element-wise Multiplication
F = A .* B;
disp("========== ELEMENT-WISE MULTIPLICATION (A .* B) ==========");
disp(F);

// Transpose of Matrix A
TA = A';
disp("========== TRANSPOSE OF A ==========");
disp(TA);

// Transpose of Matrix B
TB = B';
disp("========== TRANSPOSE OF B ==========");
disp(TB);

// Determinant of Matrix A
detA = det(A);
disp("========== DETERMINANT OF A ==========");
disp(detA);

// Determinant of Matrix B
detB = det(B);
disp("========== DETERMINANT OF B ==========");
disp(detB);

// Inverse of Matrix A
invA = inv(A);
disp("========== INVERSE OF A ==========");
disp(invA);

// Inverse of Matrix B
invB = inv(B);
disp("========== INVERSE OF B ==========");
disp(invB);

// Size of Matrix A
sizeA = size(A);
disp("========== SIZE OF A ==========");
disp(sizeA);

// Size of Matrix B
sizeB = size(B);
disp("========== SIZE OF B ==========");
disp(sizeB);

// Identity Matrix
I = eye(3,3);
disp("========== IDENTITY MATRIX ==========");
disp(I);

// Zero Matrix
Z = zeros(3,3);
disp("========== ZERO MATRIX ==========");
disp(Z);

// Ones Matrix
O = ones(3,3);
disp("========== ONES MATRIX ==========");
disp(O);