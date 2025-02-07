function cubic_spline_interpolation(s, q, s_dot, v1, v_end)
    % CUBIC_SPLINE_INTERPOLATION: Calcola la spline cubica per un giunto rotazionale.
    %
    % Sintassi:
    %   cubic_spline_interpolation(s, q, s_dot, v1, v_end)
    %
    % Input:
    %   s     - vettore dei nodi (parametro) (default: [0, 0.25, 0.5, 1])
    %   q     - vettore dei valori (angoli in gradi) corrispondenti ai nodi
    %           (default: [70, -45, 10, 100])
    %   s_dot - velocità costante lungo il parametro s (ds/dt) (default: 0.4)
    %   v1    - derivata iniziale q'(s1). Se non fornita, viene calcolata come:
    %             (q(2)-q(1))/(s(2)-s(1))
    %   v_end - derivata finale q'(s_end). Se non fornita, viene calcolata come:
    %             (q(end)-q(end-1))/(s(end)-s(end-1))
    %
    % La funzione calcola inoltre:
    %   - i coefficienti dei polinomi cubici per ciascun intervallo (salvati in una cella)
    %   - il tempo totale T di percorrenza, dato s_dot
    %   - le velocità articolari iniziale e finale (scalate da s_dot)
    %   - il massimo valore assoluto dell'accelerazione e l'intervallo in cui si verifica.
    
    % Imposta i valori di default se non sono passati come input
    if nargin < 1 || isempty(s)
        s = [0, 0.25, 0.5, 1];
    end
    if nargin < 2 || isempty(q)
        q = [70, -45, 10, 100];
    end
    if nargin < 3 || isempty(s_dot)
        s_dot = 0.4;
    end
    if nargin < 4 || isempty(v1)
        v1 = (q(2) - q(1)) / (s(2) - s(1));
    end
    if nargin < 5 || isempty(v_end)
        v_end = (q(end) - q(end-1)) / (s(end) - s(end-1));
    end

    % Numero di nodi e preallocazione
    n = length(s);
    A = zeros(n-2, n-2); % matrice dei coefficienti per i nodi interni
    b = zeros(n-2, 1);   % vettore dei termini noti

    % Costruzione del sistema per i nodi interni (v2, v3, ...)
    for i = 2:n-1
        delta_prev = s(i) - s(i-1);
        delta_next = s(i+1) - s(i);
        
        % Popola la matrice A:
        % Per i nodi interni a quelli di bordo, l'equazione di continuità della derivata
        % seconda porta ad avere una struttura (semi-)tridiagonale.
        if i < n-1
            A(i-1, i-1) = 2 * delta_prev * delta_next^2;
            A(i-1, i)   = delta_prev^2 * delta_next;
        else
            A(i-1, i-1) = 2 * delta_prev * delta_next^2;
        end
        
        % Costruzione del vettore b
        term1 = (q(i) - q(i-1)) * delta_prev^2;
        term2 = (q(i+1) - q(i)) * delta_next^2;
        b(i-1) = term1 + 2 * term2;
        
        if i == n-1
            b(i-1) = b(i-1) - v_end * delta_prev^2 * delta_next;
        end
    end
    
    % Risoluzione per le velocità interne
    if ~isempty(b)
        v_internal = A \ b;
        v = [v1; v_internal; v_end];
    else
        v = [v1; v_end];
    end
    
    % Calcolo dei coefficienti per ciascun intervallo (salvati in una cella)
    coefficients = cell(n-1, 1);
    % La forma dei polinomi adottata è:
    %   q(s) = q(seg) + a1*(s-s(seg))/delta_s + a2*((s-s(seg))/delta_s)^2 + a3*((s-s(seg))/delta_s)^3
    % dove:
    %   a1 = v(seg) * delta_s
    %   a2 = (q(seg+1)-q(seg)) - v(seg+1) * delta_s
    %   a3 = v(seg+1) * delta_s - (q(seg+1)-q(seg))
    for seg = 1:n-1
        delta_s = s(seg+1) - s(seg);
        a1 = v(seg) * delta_s;
        a2 = (q(seg+1) - q(seg)) - v(seg+1) * delta_s;
        a3 = v(seg+1) * delta_s - (q(seg+1) - q(seg));
        coefficients{seg} = [a1, a2, a3];
    end
    
    % Visualizzazione dei coefficienti per ciascun segmento
    disp('Coefficients for each segment (a1, a2, a3):');
    for seg = 1:n-1
        fprintf('Segment %d: ', seg);
        disp(coefficients{seg});
    end
    
    %% Calcoli relativi al tempo e alle velocità articolari
    T = (s(end) - s(1)) / s_dot;
    fprintf('Total traveling time: %.2f s\n', T);
    
    q_dot_start = v(1) * s_dot;
    q_dot_end = v(end) * s_dot;
    fprintf('Initial joint velocity: %.2f deg/s\n', q_dot_start);
    fprintf('Final joint velocity:   %.2f deg/s\n', q_dot_end);
    
    %% Calcolo del massimo valore assoluto dell'accelerazione
    % Per ogni segmento, se il polinomio è:
    %   q(s) = q(seg) + a1*( (s-s(seg))/delta_s ) + a2*( (s-s(seg))/delta_s )^2 + a3*( (s-s(seg))/delta_s )^3,
    % allora la derivata seconda rispetto a s è:
    %   q''(s) = 2*a2/delta_s^2 + 6*a3*( (s-s(seg))/delta_s )/delta_s^2.
    % L'accelerazione temporale è: q¨(t) = q''(s) * (ds/dt)^2.
    max_accel = 0;
    t_accel = NaN;  % istante (in s, parametrico) in cui si raggiunge l'accelerazione massima
    for seg = 1:n-1
        delta_s = s(seg+1) - s(seg);
        a = coefficients{seg};
        
        % Valutiamo q''(s) agli estremi del segmento
        q_ddot_start = (2 * a(2)) / (delta_s^2) * s_dot^2;
        q_ddot_end   = (2 * a(2) + 6 * a(3)) / (delta_s^2) * s_dot^2;
        
        [local_max, idx] = max(abs([q_ddot_start, q_ddot_end]));
        if local_max > max_accel
            max_accel = local_max;
            % Calcola il valore del parametro s all'estremo scelto
            if idx == 1
                t_accel = s(seg);
            else
                t_accel = s(seg+1);
            end
        end
    end
    fprintf('Maximum acceleration: %.2f deg/s²\n', max_accel);
    fprintf('Acceleration maximum occurs at s = %.2f (parametric value)\n', t_accel);
end
