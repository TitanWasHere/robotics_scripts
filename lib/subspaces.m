function[subspace] = subspaces(Js)
    [n_rows, n_col] = size(Js);

       
    dim_range = rank(Js);
    fprintf("[rank]: %d\n", dim_range)

    % dim. of null space
    dim_null = n_col - rank(Js);
    fprintf("[dim_null]: %d\n", dim_null)

    % null space --> vel. ai joint che danno vel 0 all'ee
    null_space = simplify(null(transpose(Js)));
    fprintf("[NULL SPACE]:")
    display(null_space);

    

    % TO FIND RANGE SPACE: TAKE THE 'dim_range' ROWS MORE CONVENIENT AND WE
    % PUT THEM INTO THE RANGE SPACE (maybe with R(J) = span { (...), (...)}
    % --> vel. che vengono mandate dai joint all'ee
end