syms q1 q2 q3 a1 a2 a3 d1
% theta alpha a d

matrix = [
 %   q1,pi/2,a1,d1;
    q2,0,a2,0;
    q3,pi/2,a3,0;
];

T = DH_HTM(matrix, "r")
