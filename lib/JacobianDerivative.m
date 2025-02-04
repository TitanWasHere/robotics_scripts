function J_dot = JacobianDerivative(J, q, q_dot)
    % JacobianDerivative computes the time derivative of a given Jacobian matrix.
    %
    % Inputs:
    %   J     : Jacobian matrix (symbolic or function handle)
    %   q     : Joint positions (symbolic or numeric vector)
    %   q_dot : Joint velocities (symbolic or numeric vector)
    %
    % Output:
    %   J_dot : Time derivative of the Jacobian matrix

    % Se q e q_dot non sono simbolici, li converto in simbolici
    if ~isa(q, 'sym')
        q = sym(q);
    end
    if ~isa(q_dot, 'sym')
        q_dot = sym(q_dot);
    end

    % Assumo che il numero di giunti sia pari alla lunghezza di q
    n = length(q);
    
    % Inizializza J_dot come matrice simbolica della stessa dimensione di J
    J_dot = sym(zeros(size(J)));
    
    % Calcola la derivata temporale di ogni elemento di J
    for i = 1:size(J,1)
        for j = 1:size(J,2)
            % Calcolo la derivata totale rispetto al tempo come somma delle derivate parziali
            % rispetto a ciascun q_k moltiplicate per q_dot(k)
            temp = sym(0);
            for k = 1:n
                temp = temp + diff(J(i,j), q(k)) * q_dot(k);
            end
            
            J_dot(i,j) = collect(temp, q_dot);
            J_dot(i,j) = simplify(temp);
        end
    end
    
end
