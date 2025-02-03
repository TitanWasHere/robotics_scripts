 
function R = RotY(angle)
    % Rotazione attorno all'asse Y
    % Input: angle (può essere numerico o simbolico)
    % Output: matrice di rotazione 3x3 attorno a Y

    R = [ cos(angle), 0, sin(angle);
                0, 1,      0;
         -sin(angle), 0, cos(angle)];
end