function R = mpt_call_cdd(S)
%
% header file to be inserted from XML source

global MPTOPTIONS
if isempty(MPTOPTIONS) && ~S.test
    MPTOPTIONS = mptopt;
end


% speed hack
if isfield(S, 'quicklp')
	% CDD doesn't like unbounded problems, therefore we introduce
	% artificial +/- 1e9 bounds (as in MPT2)
	I = eye(S.n);
	b = 1e8*ones(2*S.n, 1); % cdd doesn't like unbounded problems
	H.A = [S.Ae; S.A; I; -I];
	H.B = [S.be; S.b; b];
	if ~isempty(S.ub)
		% add bounds
		H.A = [H.A; I; -I];
		H.B = [H.B; S.ub; -S.lb];
    end
    H.A = full(H.A);
    H.B = full(H.B);
	H.lin = 1:S.me;
	H.obj = full(S.f(:)');
	R = cddmex('solve_lp',H);
	R.obj = R.objlp;
	if R.how==1
		R.how = 'ok';
		R.exitflag = MPTOPTIONS.OK;
	elseif R.how<=5
		R.how = 'infeasible';
		R.exitflag = MPTOPTIONS.INFEASIBLE;
	elseif R.how<=7
		R.how = 'unbounded';
		R.exitflag = MPTOPTIONS.UNBOUNDED;
	else
		R.how = 'other error';
		R.exitflag = MPTOPTIONS.ERROR;
	end
	return
end

if ~strcmpi(S.problem_type,'LP')
    error('mpt_call_cdd: CDD solver does not solve %s problems!',S.problem_type);
end

% objective function is zero when empty
if isempty(S.f)
    S.f = zeros(S.n,1);
end

% convert sparse matrices to full matrices
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

% merge constraints
H.A=[S.Ae; S.A];
H.B=[S.be; S.b];

% merge lb/ub with inequality constraints
% detect Inf boundaries
if S.test
    ilb = (S.lb==-Inf) | (S.lb<=-1e6);
    iub = (S.ub==Inf)  | (S.ub>=1e6);
else
    ilb = (S.lb==-Inf) | (S.lb<=-MPTOPTIONS.infbound);
    iub = (S.ub==Inf)  | (S.ub>=MPTOPTIONS.infbound);
end
% store kept rows
kept_rows.lb = find(~ilb);
kept_rows.ub = find(~iub);
if any(~ilb)
    % put ones at the positions where there is lb/ub
    Alb = zeros(nnz(~ilb),S.n);
    Alb(:,~ilb) = -eye(nnz(~ilb));
    H.A = [H.A; Alb];
    H.B = [H.B; -S.lb(~ilb)];
end
if any(~iub)
    Aub = zeros(nnz(~iub),S.n);
    Aub(:,~iub) = eye(nnz(~iub));
    H.A = [H.A; Aub];
    H.B = [H.B; S.ub(~iub)];
end
H.obj=S.f(:)';

% prepare indices of equality constraints in H.A
H.lin = 1:S.me;

% dual method is preferred
if any(strcmpi(S.solver,{'cdd.cris-cross','cdd.criss-cros','cdd.cris-cros','cdd.criss-cross','criss-cross','cris-cross','criss-cros','cris-cros'}))
    % Criss-Cross Method
    Q=cddmex('solve_lp',H);    
else
    % Dual Simplex Method
    Q=cddmex('solve_lp_DS',H);  
end

R.xopt=Q.xopt;
R.obj=Q.objlp;

% assign Lagrange multipliers
lambda=-Q.lambda;
R.lambda.ineqlin = lambda(S.me+1:S.me+S.m);
R.lambda.eqlin = lambda(1:S.me);
if ~isempty(S.lb)
    R.lambda.lower = zeros(S.n,1);
    R.lambda.lower(kept_rows.lb) = lambda(S.me+S.m+1:S.me+S.m+numel(kept_rows.lb));
else
    R.lambda.lower = zeros(S.n,1);
end
if ~isempty(S.ub) && isempty(S.lb)
    R.lambda.upper = zeros(S.n,1);
    R.lambda.upper(kept_rows.ub) = lambda(S.me+S.m+1:S.me+S.m+numel(kept_rows.ub));
elseif ~isempty(S.ub) && ~isempty(S.lb)
    R.lambda.upper = zeros(S.n,1);
    R.lambda.upper(kept_rows.ub) = lambda(S.me+S.m+numel(kept_rows.lb)+1:S.me+S.m+numel(kept_rows.lb)+numel(kept_rows.ub));
else
    R.lambda.upper = zeros(S.n,1);
end


if Q.how==0,
    R.how = 'undecided';
    if S.test
        R.exitflag = 2;
    else
        R.exitflag = MPTOPTIONS.INFEASIBLE;
    end
elseif Q.how == 1
    R.how = 'ok';
    if S.test
        R.exitflag = 1;
    else
        R.exitflag = MPTOPTIONS.OK;
    end
elseif Q.how <= 5,
    % 3 = dual-inconsistent solution
    R.how = 'infeasible';
    if S.test
        R.exitflag = 2;
    else
        R.exitflag = MPTOPTIONS.INFEASIBLE;
    end
elseif Q.how==6 || Q.how==7,
    R.how = 'unbounded';
    if S.test
        R.exitflag = 3;
    else
        R.exitflag = MPTOPTIONS.UNBOUNDED;
    end
else
    R.how= 'other error code in "exitflag" from cdd';
    if S.test
        R.exitflag = -1;
    else
        R.exitflag = MPTOPTIONS.ERROR;
    end    
end
