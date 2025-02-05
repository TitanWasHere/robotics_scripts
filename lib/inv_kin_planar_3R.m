function [q1, q2, q3] = inv_kin_planar_3R(l1, l2, l3, p, phi, sign_sin_pos_or_neg)
            if nargin < 6
                sign_sin_pos_or_neg = "pos";
            end
            % p = [px_d py_d pz_d]'
            % phi = angle end effector wrt robot's base (phi = q1+q2+q3)
            
            % To get inverse kinematic we decompose computation in two
            % parts as described in sept 2020 exam ex 3

            pt2 = p - [l3*cos(phi) l3*sin(phi) 0]'; %pos third link's base
     
            [q1, q2] = inv_kin_2R(l1, l2, pt2, sign_sin_pos_or_neg);
            
            q3 = phi - q1 - q2;
        end