syms a b c d d_i d_f T delta_phi

% Equazioni
eq1 = d == 0;
eq2 = c == (d_i * T) / delta_phi;
eq3 = a == 1 - b - c;
eq4 = b == ((d_f * T) / delta_phi - c - 3*a) / 2;

% Risoluzione delle equazioni
sol = solve([eq1, eq2, eq3, eq4], [a, b, c, d]);

% Risultati simbolici
a_sol = sol.a;
b_sol = sol.b;
c_sol = sol.c;
d_sol = sol.d;

disp('Soluzione per a:');
disp(a_sol);

disp('Soluzione per b:');
disp(b_sol);

disp('Soluzione per c:');
disp(c_sol);

disp('Soluzione per d:');
disp(d_sol);
x = -1.5708 + 4.7124 *(0.5^2) - 3.1416 * (0.5^3)
