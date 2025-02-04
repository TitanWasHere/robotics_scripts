function J_dot = JacobianDerivative(J, q, q_dot)
    % computeJacobianDerivative computes the time derivative of the Jacobian.
    %
    % Inputs:
    %   J: Jacobian matrix (symbolic or function handle)
    %   q: Joint positions (symbolic or numeric)
    %   q_dot: Joint velocities (symbolic or numeric)
    %
    % Output:
    %   J_dot: Time derivative of the Jacobian

    % Ensure q and q_dot are symbolic for differentiation
    if ~isa(q, 'sym')
        q = sym(q);
    end
    if ~isa(q_dot, 'sym')
        q_dot = sym(q_dot);
    end

    % Compute the time derivative of the Jacobian
    J_dot = sym(zeros(size(J))); % Initialize J_dot as a symbolic matrix
    for i = 1:size(J, 1)
        for j = 1:size(J, 2)
            % Differentiate each element of J with respect to time
            J_dot(i, j) = diff(J(i, j), q(1)) * q_dot(1) + diff(J(i, j), q(2)) * q_dot(2);
        end
    end
end