syms l1 l2 q1 q2



% Intervallo dei valori di s
    s = linspace(0, 1, 1000); % Puoi modificare il range [-2, 2]

    % Definizione di q1(s) e q2(s)
    q1 = -1.7899 - 2.4305 * s + 8.2308 * s.^2 - 4.0104 * s.^3;
    q2 = 2.4039 + 1.7841 * s - 2.0674 * s.^2 - 0.5498 * s.^3;

    % Creazione del grafico
    figure;
    plot(s, q1, 'b', 'LineWidth', 1.5); hold on;
    plot(s, q2, 'r', 'LineWidth', 1.5);

    % Personalizzazione del grafico
    xlabel('s');
    ylabel('q(s)');
    title('Plot di q_1(s) e q_2(s)');
    legend('q_1(s)', 'q_2(s)');
    grid on;




p = [
    l1 * cos(q1) + l2 * cos(q1+q2);
    l1 * sin(q1) + l2 * sin(q1+q2);
];

J = jacobian(p, [q1,q2]);
J = subs(J, [l1,l2], [1,1])
detJ = det(J);
simplify(detJ);
p = subs(p, [l1,l2], [1,1]);

p_i = [0.6, -0.4];
p_f = [1,1];

i_c2 = (p_i(1) ^ 2 + p_i(2) ^ 2 - 2) / 2;
i_s2 = sqrt(1-i_c2^2);
i_q2 = atan2(i_s2, i_c2)

i_q1 = atan2(p_i(2), p_i(1)) - atan2(i_s2, 1 + i_c2)

f_c2 = (p_f(1) ^ 2 + p_f(2) ^ 2 - 2) / 2;
f_s2 = sqrt(1-f_c2^2);
f_q2 = atan2(f_s2, f_c2)

f_q1 = atan2(p_f(2), p_f(1)) - atan2(f_s2, 1 + f_c2)

J_qi = subs(J, [q1,q2], [i_q1, i_q2])
J_qf = subs(J, [q1,q2], [f_q1, f_q2])
