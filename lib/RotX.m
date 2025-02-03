 function R = RotX(angle)
    % Rotazione attorno all'asse X
    % Input: angle (può essere numerico o simbolico)
    % Output: matrice di rotazione 3x3 attorno a X

    R = [1,      0,           0;
         0, cos(angle), -sin(angle);
         0, sin(angle),  cos(angle)];
end