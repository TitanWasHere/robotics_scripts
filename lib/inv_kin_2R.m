function [q1,q2] = inv_kin_2R(l1, l2, p, sign_sin_pos_or_neg)
            % p = [px_d py_d pz_d]'
            if nargin < 4
                sign_sin_pos_or_neg = "pos";
            end
    
            if sign_sin_pos_or_neg == "neg"
                sign = -1;
            else
                sign = 1;
            end
            
            px = p(1);
            py = p(2);
     
            cos_2 = (px^2 + py^2 -l1^2 - l2^2)/(2*l1*l2);
%             if cos_2 > 1 || cos_2 < -1
%                 q1 = -1000;
%                 q2 = -1000;
%                 fprintf("no solutions")
%                 return 
%             end
            
            sin_2 = sign*sqrt(1-cos_2^2);
            q2 = atan2(sin_2, cos_2);
            
            det_1 = l1^2 + l2^2 + 2*l1*l2*cos(q2);
                    
            if det_1 ~= 0
                sin_1 = (py*(l1+l2*cos(q2))-px*l2*sin(q2))/det_1;
                cos_1 = (px*(l1+l2*cos(q2))+py*l2*sin(q2))/det_1;
                q1 = atan2(sin_1, cos_1);
            else
                fprintf("INFO: det_1 == 0, so alternative method will be used")
                q1 = atan2(py, px) - atan2(l2*sin_2, l1+l2*cos_2);
                q1 = wrapToPi(q1); %express q1 in [-pi, pi]
            end
    fprintf("\nq1: ")
    disp(q1)
    fprintf("q2: ")
    disp(q2)
end

