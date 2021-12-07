function R = mpt_call_cplex(S)
%
% header file to be inserted from XML source

global MPTOPTIONS
if isempty(MPTOPTIONS) && ~S.test
    MPTOPTIONS = mptopt;
end

if strcmpi(S.problem_type,'LCP')
    error('mpt_call_lcp: CPLEX solver does not solve %s problems!',S.problem_type);
end

    
% CPLEX interface by IBM
if exist('Cplex','file')==6
    % overwrite default settings
    % 0 Automatic
    % 1 Primal Simplex
    % 2 Dual Simplex
    % 3 Network Simplex
    % 4 Barrier
    % 5 Sifting
    % 6 Concurrent
    % prefer dual-simplex method because with automatic method some tests
    % fail (mostly polyhedron_extreme)
    if S.test
        matlab_version = version('-release');
        if str2double(matlab_version(1:4))>=2016
            % for CPLEX 12.7
            options=cplexoptimset;
            options.Display = 'off';
        else
            options=cplexoptimset('cplex');
        end
    else
        options = MPTOPTIONS.modules.solvers.cplex;
    end
    
    if strcmpi(S.problem_type,'LP')
        % direct call to cplexlp
        [R.xopt,R.obj,exitflag,OUTPUT,R.lambda]=cplexlp(S.f,S.A,S.b,S.Ae,S.be,S.lb,S.ub,S.x0,options);
    elseif strcmpi(S.problem_type,'QP')
        % direct call to cplexqp
        [R.xopt,R.obj,exitflag,OUTPUT,R.lambda]=cplexqp(S.H,S.f,S.A,S.b,S.Ae,S.be,S.lb,S.ub,S.x0,options);
        
    elseif strcmpi(S.problem_type,'MILP')
        % direct call to cplexmilp
        [R.xopt,R.obj,exitflag,OUTPUT] = cplexmilp(S.f,S.A,S.b,S.Ae,S.be,[],[],[],S.lb,S.ub,S.vartype',S.x0,options);
        R.lambda = [];
    elseif strcmpi(S.problem_type,'MIQP')
        % direct call to cplexmiqp
        [R.xopt,R.obj,exitflag,OUTPUT] = cplexmiqp(S.H,S.f,S.A,S.b,S.Ae,S.be,[],[],[],S.lb,S.ub,S.vartype',S.x0,options);
        R.lambda = [];
    end
    
    if isempty(R.xopt)        
        % in case no output is returned, make zero output to at least match the
        % dimension
        R.xopt = S.x0;
    end
    if isempty(R.obj)
        R.obj = 0;
    end

    if isempty(R.lambda)
        % in case of no lambda, create empty structure
        R.lambda.ineqlin = [];
        R.lambda.eqlin = [];
        R.lambda.lower = [];
        R.lambda.upper = [];
    else
        c=Cplex;
        % version 12 has opposite signs
        if strcmp(c.Version,'12.2.0.0')
            R.lambda.ineqlin = -R.lambda.ineqlin;
            R.lambda.eqlin = -R.lambda.eqlin;
        end 
    end

    
    switch OUTPUT.cplexstatus
        case {1,5,15,17,19,101,102,115,121,123,125}
            R.how = 'ok';
            if S.test
                R.exitflag = 1;
            else
                R.exitflag = MPTOPTIONS.OK;
            end
        case {4,119}
            % if the result is unbounded or infeasible, put bounds on all variables
            % and check unbounded flag;
            if S.test
                lb = -1e6*ones(S.n,1);
                ub = 1e6*ones(S.n,1);
            else
                lb = -MPTOPTIONS.infbound*ones(S.n,1);
                ub = MPTOPTIONS.infbound*ones(S.n,1);
            end
            if strcmpi(S.problem_type,'LP')
                [~,~,ef]=cplexlp(S.f,S.A,S.b,S.Ae,S.be,lb,ub,S.x0,options);
            elseif strcmpi(S.problem_type,'QP')
                [~,~,ef]=cplexqp(S.H,S.f,S.A,S.b,S.Ae,S.be,lb,ub,S.x0,options);
            elseif strcmpi(S.problem_type,'MILP')
                [~,~,ef] = cplexmilp(S.f,S.A,S.b,S.Ae,S.be,[],[],[],lb,ub,S.vartype',S.x0,options);
            elseif strcmpi(S.problem_type,'MIQP')
                [~,~,ef] = cplexmiqp(S.H,S.f,S.A,S.b,S.Ae,S.be,[],[],[],lb,ub,S.vartype',S.x0,options);
            end
            if ef==1
                % optimal -> the problem is unbounded
                if S.test
                    R.exitflag = 3;
                else
                    R.exitflag = MPTOPTIONS.UNBOUNDED;
                end
                R.how = 'unbounded';
            else
                R.how = 'infeasible';
                if S.test
                    R.exitflag = 2;
                else
                    R.exitflag = MPTOPTIONS.INFEASIBLE;
                end
            end
        case {2,118}
            if S.test
                R.exitflag = 3;
            else
                R.exitflag = MPTOPTIONS.UNBOUNDED;
            end
            R.how = 'unbounded';
        case {3,6,103}
            R.how = 'infeasible';
            if S.test
                R.exitflag = 2;
            else
                R.exitflag = MPTOPTIONS.INFEASIBLE;
            end      
        otherwise
            R.how = OUTPUT.cplexstatusstring;
            if S.test
                R.exitflag = -1;
            else
                R.exitflag = MPTOPTIONS.ERROR;
            end                  
    end
    

elseif exist('cplexint','file')==3
    % cplexint for CPLEX >9
    A = [S.Ae; S.A];
    b = [S.be; S.b];

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
        Alb(:,~ilb) = -speye(nnz(~ilb));
        A = [A; Alb];
        b = [b; -S.lb(~ilb)];
    end
    if any(~iub)
        Aub = zeros(nnz(~iub),S.n);
        Aub(:,~iub) = speye(nnz(~iub));
        A = [A; Aub];
        b = [b; S.ub(~iub)];
    end
    
    
    INDEQ = (1:S.me)';  % indices of equality constraints
    if S.test
        OPTIONS.verbose = 0;
        OPTIONS.lic_rel = 1e2;  % after how many runs to release the license
    else
        OPTIONS = MPTOPTIONS.modules.solvers.cplexint;
    end
    PARAM.int=[1063, 2];    % use the dual-simplex method
    
    [R.xopt,R.obj,SOLSTAT,DETAILS] = cplexint(S.H, S.f(:), A, b, INDEQ, [], [], [], S.vartype, PARAM, OPTIONS);
        
    if S.test
        R.exitflag = 2;
    else
        R.exitflag = MPTOPTIONS.INFEASIBLE;
    end

    lambda = -DETAILS.dual;
    if ~isempty(lambda)
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
    else
        R.lambda = lambda;
    end
    
    R.how = lower(DETAILS.statstring);
    if strcmp(R.how, 'optimal') || strcmp(R.how, 'optimalrelaxed') || ...
            strcmp(R.how, 'optimaltol') || strcmp(R.how, 'integer optimal solution') || ...
            strcmp(R.how, 'optimal with unscaled infeasibilities')
        R.how = 'ok';
        if S.test
            R.exitflag = 1;
        else
            R.exitflag = MPTOPTIONS.OK;
        end
    elseif strcmp(R.how,'unbounded or infeasible')
        % if the result is unbounded or infeasible, put bounds on all variables
        % and check unbounded flag;
        if S.test
            lb = -1e6*ones(S.n,1);
            ub = 1e6*ones(S.n,1);
        else
            lb = -MPTOPTIONS.infbound*ones(S.n,1);
            ub = MPTOPTIONS.infbound*ones(S.n,1);
        end
        [~,~,~,D] = cplexint(S.H, S.f(:), A, b, INDEQ, [], lb, ub, S.vartype, PARAM, OPTIONS);
        if strcmp(D.statstring, 'optimal') || strcmp(D.statstring, 'optimalrelaxed') || ...
                strcmp(D.statstring, 'optimaltol') || strcmp(D.statstring, 'integer optimal solution') || ...
                strcmp(D.statstring, 'optimal with unscaled infeasibilities')
            % optimal -> the problem is unbounded
            if S.test
                R.exitflag = 3;
            else
                R.exitflag = MPTOPTIONS.UNBOUNDED;
            end
            R.how = 'unbounded';
        else
           R.how = 'infeasible';
           if S.test
               R.exitflag = 2;
           else
               R.exitflag = MPTOPTIONS.INFEASIBLE;
           end
        end
    end
        
end
    
end
