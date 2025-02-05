function [q1, q2] = inv_kin_RP( p, sign_sin_pos_or_neg)
            % p = [px_d py_d pz_d]'
            if nargin < 2
                sign_sin_pos_or_neg = "pos";
            end
    
            if sign_sin_pos_or_neg == "neg"
                sign = -1;
            else
                sign = 1;
            end
            
            px = p(1);
            py = p(2);
     
            q2 = sign*sqrt(px^2 + py^2);
            q1 = atan2(py/q2, px/q2);
        end