function[Js] = singularities(J, joints)
    [n_rows, n_col] = size(J);
    if n_rows ~= n_col
        fprintf("[WARNING]: matrix not squared, calulating J(q) * J(q)^T\n")
        J_new = J * transpose(J);
        det_J = simplify(det(J_new));
        fprintf("[det]: %s\n", det_J);
        if det_J == 0
            J_new = transpose(J) * J
            det_J = simplify(det(J_new));
            fprintf("[det]: %s\n", det_J);
        end
    end

    
    
    pause
    % es.
    Js = subs(J, joints(2), 0);
    display(Js)
    
    % dim. of null space
    dim_null = n_col - rank(Js);
    fprintf("[dim_null]: %d\n", dim_null)

    % null space --> vel. ai joint che danno vel 0 all'ee
    null_space = simplify(null(Js));
    fprintf("[NULL SPACE]:")
    display(null_space);

    % dim. of range space:
    dim_range = rank(Js);
    display(dim_range);

    % TO FIND RANGE SPACE: TAKE THE 'dim_range' COLS MORE CONVENIENT AND WE
    % PUT THEM INTO THE RANGE SPACE (maybe with R(J) = span { (...), (...)}
    % --> vel. che vengono mandate dai joint all'ee
end