function[singularity_values] = singularities(J, joints)
    [n_rows, n_col] = size(J);
    if n_rows ~= n_col
        fprintf("[WARNING]: matrix not squared, calulating J(q) * J(q)^T")
        J = J * transpose(J);
    end

    det_J = simplify(det(J));

    fprintf("[det]: %s", det_J);
    
    % es.
    Js = subs(J, q3, 0);
    
    % dim. of null space:
    dim_null = n_col - rank(Js);
    display(dim_null)

    % null space
    null_space = simplify(null(Js));
    display(null_space);

    % dim. of range space:
    dim_range = rank(Js);
    display(dim_range);

    % TO FIND RANGE SPACE: TAKE THE 'dim_range' ROWS MORE CONVENIENT AND WE
    % PUT THEM INTO THE RANGE SPACE (maybe with R(J) = span { (...), (...)}

end