function [pos, vel] = bang_bang_profile(T, Vmax, Amax, L, t)
    ta = Vmax / Amax; % Acceleration time
    L_acc = 0.5 * Amax * ta^2; % Distance covered during acceleration

    if 2 * L_acc > L  % Triangle profile
        ta = sqrt(L / Amax);
        tc = 0;
    else  % Trapezoidal profile
        tc = (L - 2 * L_acc) / Vmax;
    end

    t1 = ta;
    t2 = ta + tc;
    t3 = 2 * ta + tc;

    pos = zeros(size(t));
    vel = zeros(size(t));

    for i = 1:length(t)
        if t(i) < t1
            pos(i) = 0.5 * Amax * t(i)^2;
            vel(i) = Amax * t(i);
        elseif t(i) < t2
            pos(i) = L_acc + Vmax * (t(i) - t1);
            vel(i) = Vmax;
        else
            pos(i) = L - 0.5 * Amax * (T - t(i))^2;
            vel(i) = Amax * (T - t(i));
        end
    end
end
