function R = mpt_call_nag(S)
%
% header file to be inserted from XML source

global MPTOPTIONS
if isempty(MPTOPTIONS) && ~S.test
    MPTOPTIONS = mptopt;
end

if ~any(strcmpi(S.problem_type,{'LP','QP'}))
    error('mpt_call_nag: NAG solver does not solve %s problems!',S.problem_type);
end

% convert sparse matrices to full matrices
if issparse(S.H),
    S.H = full(S.H);
end
if issparse(S.f)
    S.f = full(S.f);
end
if issparse(S.A)
    S.A = full(S.A);
end
if issparse(S.b)
    S.b = full(S.b);
end
if issparse(S.Ae)
    S.Ae = full(S.Ae);
end
if issparse(S.be)
    S.be = full(S.be);
end

% need to provide lb, ub if empty
if isempty(S.lb)
    if S.test
        S.lb = -1e9*ones(S.n,1);
    else
        S.lb = -MPTOPTIONS.infbound*ones(S.n,1);
    end
end
if isempty(S.ub) 
    if S.test
        S.ub = 1e9*ones(S.n,1);        
    else
        S.ub = MPTOPTIONS.infbound*ones(S.n,1);
    end
end
    

if strcmpi(S.solver,'nag.e04mcf')
    % NAG e04mcf
    % requires QP to be formulated as:
    %  min 0.5x'*H*x + f'*x
    %   s.t.:  l <= x <= u
    %          g1 <= G*x <= g2
    
    % equality constraints are treated as double-sided inequalities
    % lower bound for the left hand side of inequalities g1 <= G*x<= g2 is
    % set as very low number to achieve dimension match
    if S.test
        g1 = [S.lb; -1e9*ones(S.m,1); S.be];
    else
        g1 = [S.lb; -MPTOPTIONS.infbound*ones(S.m,1); S.be];
    end
    g2 = [S.ub; S.b; S.be];
    G = [S.A; S.Ae];
    
    % specific options can be passed through "options" structure
    %    options- (class: struct, value by default: []) options:
    %
    %      .ftol  (1e-9), the maximum acceptable violation in each constraint at a 'feasible' point.
    %      .rank_tol  (1e-13), enables the user to control the condition number of the triangular factor R
    %      .crash_tol  (0.01), a constraint of the form a^Tx >= l will be included in the initial working
    %        set if |a^Tx-l| <= options.crash_tol X (1+|l|) 0.0 < options.crash_tol <= 1.0
    %      .reset_ftol  (5),this option is part of an anti-cycling procedure designed to guarantee progress
    %        even on highly degenerate problems
    %      .max_iter  (100), maximum number of iterations to be performed
    %      .fcheck  (50), every fcheck iterations, a numerical test is made to see if the current solution
    %        x satisfies the constraints in the working set
    %      .inf_bound  (1e12), defines the 'infinite' bound in the definition of the problem constraints

    if strcmpi(S.problem_type,'QP')        
         if ~S.test
             options = MPTOPTIONS.modules.solvers.nag.qp;
             % correct initial conditions because NAG sometimes freezes    
             % S.x0 = mexnagqp([],[],G,g1,g2,S.x0,options);
             [R.xopt, R.obj, exitflag, lambda] = mexnagqp(S.H, S.f, G, g1, g2, S.x0, options);
         else
             [R.xopt, R.obj, exitflag, lambda] = mexnagqp(S.H, S.f, G, g1, g2, S.x0);
         end        
    else
        if ~S.test
            options = MPTOPTIONS.modules.solvers.nag.lp;
            [R.xopt, R.obj, exitflag, lambda] = mexnaglp(S.f, G, g1, g2, S.x0, options);
        else
            [R.xopt, R.obj, exitflag, lambda] = mexnaglp(S.f, G, g1, g2, S.x0);
        end        
    end
    
    if S.test
        activelb = (R.xopt < S.lb + 1e-4 );
        activeub = (R.xopt > S.ub - 1e-4 );        
    else
        activelb = (R.xopt < S.lb + MPTOPTIONS.rel_tol );
        activeub = (R.xopt > S.ub - MPTOPTIONS.rel_tol );
    end
    R.lambda.lower = lambda(1:S.n);
    R.lambda.lower(~activelb) = 0;
    R.lambda.upper = -lambda(1:S.n);
    R.lambda.upper(~activeub) = 0;
    R.lambda.ineqlin = -lambda(S.n+1:S.n+S.m);
    R.lambda.eqlin = -lambda(S.n+S.m+1:S.n+S.m+S.me);
    %R.lambda = lambda(S.n+1:S.n+S.m+S.me);
    
    switch exitflag,
        case 0,
            R.how = 'ok';
            if S.test
                R.exitflag = 1;
            else
                R.exitflag = MPTOPTIONS.OK;
            end
        case 291,
            R.how = 'not_unique_solution';
            % return as correct solution
            if S.test
                R.exitflag = 1;
            else
                R.exitflag = MPTOPTIONS.OK;
            end
        case 292
            R.how = 'unbounded';
            if S.test
                R.exitflag = 3;
            else
                R.exitflag = MPTOPTIONS.UNBOUNDED;
            end
        case {293, 294, 295}
            R.how = 'infeasible';
            if S.test
                R.exitflag = 2;
            else
                R.exitflag = MPTOPTIONS.INFEASIBLE;
            end
        otherwise
            R.how = 'other error code in "exitflag" from e04mfc.';
            if S.test
                R.exitflag = -1;
            else
                R.exitflag = MPTOPTIONS.ERROR;
            end
    end
    
else
	% e04nf or e04mf
	
	% equality constraints are treated as double-sided inequalities
    % lower bound for the left hand side of inequalities g1 <= G*x<= g2 is
    % set as very low number to achieve dimension match
    if S.test
        g1 = [S.lb; -1e9*ones(S.m,1); S.be];
    else
        g1 = [S.lb; -MPTOPTIONS.infbound*ones(S.m,1); S.be];
    end
    g2 = [S.ub; S.b; S.be];
    G = [S.A; S.Ae];

	istate = zeros(length(g1), 1, 'int64');
	if strcmpi(S.problem_type,'QP')
		% initialize NAG
		[cwsav,lwsav,iwsav,rwsav,ifail] = nag_opt_init('nag_opt_qp_dense_solve');
		% E04NF for QPs
		[istate, R.xopt, iter, R.obj, ax, lambda, user, ...
			lwsav, iwsav, rwsav, ifail] = e04nf(G, g1, g2, S.f, S.H, ...
			@e04nf_qphess, istate, S.x0, lwsav, iwsav, rwsav);
	else
		% initialize NAG
		[cwsav,lwsav,iwsav,rwsav,ifail] = nag_opt_init('nag_opt_lp_solve');
		% E04MF for LPs
		[istate, R.xopt, iter, R.obj, ax, lambda, ...
			lwsav, iwsav, rwsav, ifail] = e04mf(G, g1, g2, S.f, istate, ...
			S.x0, lwsav, iwsav, rwsav);
    end
    
    if S.test
        activelb = (R.xopt < S.lb + 1e-4 );
        activeub = (R.xopt > S.ub - 1e-4 );        
    else
        activelb = (R.xopt < S.lb + MPTOPTIONS.rel_tol );
        activeub = (R.xopt > S.ub - MPTOPTIONS.rel_tol );
    end
    R.lambda.lower = lambda(1:S.n);
    R.lambda.lower(~activelb) = 0;
    R.lambda.upper = -lambda(1:S.n);
    R.lambda.upper(~activeub) = 0;
    R.lambda.ineqlin = -lambda(S.n+1:S.n+S.m);
    R.lambda.eqlin = -lambda(S.n+S.m+1:S.n+S.m+S.me);
	
	% decide status
	switch ifail,
		case {0, 1, 4},
			% optimal
			R.how = 'ok';
            if S.test
                R.exitflag = 1;
            else
                R.exitflag = MPTOPTIONS.OK;
            end
		case 2,
			% unbounded
			R.how = 'unbounded';
			if S.test
				R.exitflag = 3;
            else
                R.exitflag = MPTOPTIONS.UNBOUNDED;
            end
		case 3
            R.how = 'infeasible';
            if S.test
                R.exitflag = 2;
            else
                R.exitflag = MPTOPTIONS.INFEASIBLE;
            end
        otherwise
            R.how = 'other error code in "exitflag" from NAG.';
            if S.test
                R.exitflag = -1;
            else
                R.exitflag = MPTOPTIONS.ERROR;
            end
	end
	
end

end

%--------------------------------------
function [hx, user, iwsav] = e04nf_qphess(n, jthcol, h, ldh, x, user, iwsav)

hx = h*x;

end
