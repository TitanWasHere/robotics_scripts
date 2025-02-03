function[q,varargout]=Newton_method(q0,J,pd,p,eps,n_iter_max,varargin)
% DAVIDE'S SCRIPT!
% this function uses the Newton algorithm to estimate q_k which realise the
% inverse kinematics, i.e. finding the variables q1,q2...,qn that realize a
% desired pose p_d of the end-effector, given of course the direct 
% kinematics of the problem p(q)

% the INPUT required are ....many... :
% q0 = initial guess of the algorithm (vector of dimension n, number of joints
% J = analityc jacobian of the robot (differentiation of direct kinemtics p(q)
% pd = desired pos of EE (the task vector of dimension m, the components 
%       are real numbers)
% p = direct kinematiks of the robot ( p(q), vector of dimension m, the 
%       components have q1,...qn in it)
% eps = posterior stopping criterion, error e_k= |pd-p_k(q)|. [it's the max
%       tollerance between the desired value pd and the estimated value p, 
%       with the k-th p(q)]
% n_iter = prior stopping criterion, max number of iterations for the
%       algorithm. When it reaches n_iter_max iterations, it stops with a 
%       failure message

% the optional INPUT (not required) are:
% epsJ = max tolerance for the determinant of J. If |det(J)|<=epsJ. This
%       function will use the pseudo-inverse (det(J) is too small in this
%       case)
% eps_q = max tolerance for updating the value of sucessive estimations of
%       joint variables q. This function will stop iterating the algorithm
%       if the variation of q_k+1 respect the previous estimation q_k is 
%       too small
 
%% VERY IMPORTANT INFO
% If in your jacobian appear other constant variables such as L1,L2..a1,
% and so on, you have to initialize those variables here in this script
% before running it with your data (*) [down]


syms q1 q2 q3 q4 q5 q6 real

% initialize variables
err = eps+1; 
n_iter = 0;
numJ=J;
invJ = simplify(inv(J));
n=length(q0);
m=length(p);
n_pseudo=[];
d=[];
q_fin=q0;
p_fin=zeros(m,1);
% (*)
% [here] you can set your variables existing in the jacobian for YOUR problem
% for example, write:
% L1=0.5; 
% L2=0.4; and so on

% initialization of the optional input epsJ
switch nargin
    case 6
        epsJ = 10^-4;
        eps_q = 10^-4;
    case 7
        epsJ = varargin{1};
        eps_q = 10^-4;
    case 8
        epsJ=varargin{1};
        eps_q = varargin{2};
end

%% Real code stars here
% creating vector of estimated q
while err>eps && n_iter<n_iter_max
    if n_iter ~= 0
        q0=q;
    end
    % brute force shit to compile this other shit (IGNORE)
    switch n
        case 1
            q1=q0;
        case 2
            q1=q0(1);
            q2=q0(2);
        case 3
            q1=q0(1);
            q2=q0(2);
            q3=q0(3);
        case 4
            q1=q0(1);
            q2=q0(2);
            q3=q0(3);
            q4=q0(4);
    end
    % evaluating p_k in the current q_k, creating the vector containing all
    % p_k and avaluating J in the current q_k
    p0 = eval(p);
    p_fin(1:3,n_iter + 1) = p0;
    numJ=eval(J);
    % check det(J) and wheter using pseudo-inverse or not
    if abs(det(numJ)) <= epsJ
        inv_numJ = simplify(pinv(J));
        inv_numJ = eval(inv_numJ);
        n_pseudo= [n_psuedo n_iter+1];
    else 
        inv_numJ = eval(invJ);
    end

    % newton algorithm
    q = q0 + inv_numJ * (pd - p0);
    % another stopping criterion
    if abs(q-q0)<eps_q
        break
    end
    % updating required and optional outputs
    n_iter = n_iter+1;
    d=[d det(numJ)];
    q_fin=[q_fin q];
    err=norm(pd-p0,2);
    ERR(n_iter) = err;
end

% check if Newton algorithm worked or not
if n_iter==n_iter_max
    disp(['this function did not produce any valuable result. ' ...
        'Please, consider changing input guess'])
end

% setting optional output
switch nargout
    case 2
        varargout{1} = n_iter;
    case 3
        varargout{1} = n_iter;
        varargout{2} = q_fin;
    case 4
        varargout{1} = n_iter;
        varargout{2} = q_fin;
        varargout{3} = p_fin;
    case 5
        varargout{1} = n_iter;
        varargout{2} = q_fin;
        varargout{3} = p_fin;
        varargout{4} = ERR;
    case 6
        varargout{1} = n_iter;
        varargout{2} = q_fin;
        varargout{3} = p_fin;
        varargout{4} = ERR;
        varargout{5} = d;
end
% I tried to go further than my abilities and plotting a nice table with
% all results. MATLAB won so ignore these comments (puto)

% fprintf('iteration k\t q1_k\t q2_k\t q3_k\t px_k\t py_k\t pz_k\t ||e_k|\t detJ_k\n');
% fprintf('%d\t %5.4f\t %5.4f\t %5.4f\t %5.4f\t %5.4f\t %5.4f\t %6.5f\t %5.4f\n', ...
%     [0:n_iter; q_fin(1,:); q_fin(2,:); q_fin(3,:); p_fin(1,:); p_fin(2,:); p_fin(3,:); ERR;d])