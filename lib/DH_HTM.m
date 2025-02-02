function [DH_HTM, intermediateHTM] = DH_HTM(Matrix, angtype)
% DH_HTM computes the homogeneous transformation from frame 0 to the end-effector.
%
% Input:
%   Matrix: DH Table of size (n,4), where each row is [theta, alpha, r, d].
%   angtype: A parameter (e.g., 'deg' or 'rad') used by the helper function dh_link.
%
% Output:
%   DH_HTM: The overall homogeneous transformation matrix (4x4) from frame 0 to EE.
%   intermediateHTM: A 3D array of size (4,4,n) where intermediateHTM(:,:,i) is the 
%                    transformation from frame 0 to frame i.

    if size(Matrix,2) ~= 4
        error("Matrix must have 4 columns");
    end

    output = eye(4);
    len = size(Matrix,1); % Number of rows (links)

    % Preallocate a 3D array to store intermediate transformations.
    intermediateHTM = sym(zeros(4,4,len));  % use sym if working with symbolic expressions
    
    for i = 1 : len
        params = Matrix(i,:);
        theta = params(1);
        alpha = params(2);
        rx = params(3);
        dz = params(4);
        
        % Compute the transformation for the current link.
        next = dh_link(theta, alpha, rx, dz, angtype);
        
        % Multiply to get the transformation from frame 0 to the current frame.
        output = output * next;
        
        % Simplify and store the intermediate transformation.
        intermediateHTM(:,:,i) = simplify(output);
    end

    % Simplify the overall transformation matrix.
    DH_HTM = simplify(output);
end
