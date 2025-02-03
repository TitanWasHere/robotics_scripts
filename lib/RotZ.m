 
function R = RotZ(angle)
    % Rotazione attorno all'asse Z
    % Input: angle (può essere numerico o simbolico)
    % Output: matrice di rotazione 3x3 attorno a Z

    R = [cos(angle), -sin(angle), 0;
         sin(angle),  cos(angle), 0;
               0,           0,    1];
end