syms q1 q2 q3 a2 a3

p = [
    cos(q1)*(a2*cos(q2) + a3 * cos(q2 + q3));
    sin(q1)*(a2 * cos(q2) + a3 * cos(q2 + q3));
    a2 * sin(q2) + a3 * sin(q2+q3);
];

J = jacobian(p, [q1,q2,q3])

d = det(J);
simplify(d)

Js = [
-sin(q1)*(a3*cos(q2 + q3) + a2*cos(q2)), -cos(q1)*(a3*sin(q2 + q3) + a2*sin(q2)), -a3*sin(q2 + q3)*cos(q1);
 -cos(q1)*(a3*cos(q2 + q3) + a2*cos(q2)), -sin(q1)*(a3*sin(q2 + q3) + a2*sin(q2)), -a3*sin(q2 + q3)*sin(q1);
                                      0,            a3*cos(q2 + q3) + a2*cos(q2),          a3*cos(q2 + q3);
];
Js = subs(Js, q3, 0);

NullSpaceJ = null(Js)
dimNullSpaceJ=size(NullSpaceJ,2);
simplify(Js*NullSpaceJ)

RangeJ=orth(J)
