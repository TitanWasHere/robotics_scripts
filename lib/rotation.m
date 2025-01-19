syms alpha beta gamma 


RotX(alpha) * RotY(beta) * [0;0;1]

%Ri = [
%    0,0,1;
%    0,1,0;
%    -1,0,0;
%];

%Rf = [
%    sqrt(2)/2,0,sqrt(2)/2;
%    sqrt(2)/2,0,-sqrt(2)/2;
%    0,1,0;
%];

%Rif = transpose(Ri) * Rf;

%[sol1, sol2] = rotm2eul(Rf, "ZXZ");
%sol1
%sol2

%R = RotZ(alpha)*RotX(beta)

function R = RotX(angle)
    % Rotazione attorno all'asse X
    % Input: angle (può essere numerico o simbolico)
    % Output: matrice di rotazione 3x3 attorno a X

    R = [1,      0,           0;
         0, cos(angle), -sin(angle);
         0, sin(angle),  cos(angle)];
end

function R = RotY(angle)
    % Rotazione attorno all'asse Y
    % Input: angle (può essere numerico o simbolico)
    % Output: matrice di rotazione 3x3 attorno a Y

    R = [ cos(angle), 0, sin(angle);
                0, 1,      0;
         -sin(angle), 0, cos(angle)];
end

function R = RotZ(angle)
    % Rotazione attorno all'asse Z
    % Input: angle (può essere numerico o simbolico)
    % Output: matrice di rotazione 3x3 attorno a Z

    R = [cos(angle), -sin(angle), 0;
         sin(angle),  cos(angle), 0;
               0,           0,    1];
end