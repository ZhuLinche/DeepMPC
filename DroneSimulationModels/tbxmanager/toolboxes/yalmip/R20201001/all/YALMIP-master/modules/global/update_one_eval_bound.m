function p = update_one_eval_bound(p,i);

% This function computes upper and lower bounds on the scalar nonlinear
% nonlinear functions modelled using nonlinear evaluation-based operators.
% Given a lower bound xL and upper bound xU on x, find upper and lower
% bounds on f(x)

arg = p.evalMap{i}.variableIndex;
xL = p.lb(arg);
xU = p.ub(arg);
L = -inf;
U = inf;
if ~isempty(p.evalMap{i}.properties.bounds)
    % A bound generator is available!
    [L,U] = feval(p.evalMap{i}.properties.bounds,xL,xU,p.evalMap{i}.arg{2:end-1});
else
    if isa(p.evalMap{i}.properties.monotonicity,'char')
        monotonicity = p.evalMap{i}.properties.monotonicity;
    elseif isa(p.evalMap{i}.properties.monotonicity,'function_handle')
        monotonicity =  p.evalMap{i}.properties.monotonicity(xL,xU);
    elseif ~isempty(p.evalMap{i}.properties.stationary) && (isequal(p.evalMap{i}.properties.convexity,'convex') || isequal(p.evalMap{i}.properties.convexity,'concave'))
         monotonicity = DeriveMonotonicityFromStationary(p.evalMap{i}.properties,xL,xU);
    elseif ~isempty(p.evalMap{i}.properties.stationary) && (isequal(p.evalMap{i}.properties.shape,'bell-shape') || isequal(p.evalMap{i}.properties.shape,'v-shape'))
         monotonicity = DeriveMonotonicityFromShape(p.evalMap{i}.properties,xL,xU);         
    else
        monotonicity = 'none';
    end
    
    if strcmpi(monotonicity,'increasing')
        % No generator is available, but bound follows from monotinicity
        arg = p.evalMap{i}.arg;
        if length(arg{1}) > 1
            disp(['The ' p.evalMap{i}.fcn ' operator does not have a bound operator'])
            disp('This is required for multi-input single output operators');
            disp('Sampling approximation does not work in this case.');
            error('Missing bound operator');
        end
        arg{1} = xL;
        L = real(feval(p.evalMap{i}.fcn,arg{1:end-1}));
        if isnan(L)            
            L = p.evalMap{i}.properties.range(1);                        
        end
        arg{1} = xU;
        U = real(feval(p.evalMap{i}.fcn,arg{1:end-1}));
        if isnan(U)            
            U = p.evalMap{i}.properties.range(2);            
        end
    elseif strcmpi(p.evalMap{i}.properties.monotonicity,'decreasing')
        arg = p.evalMap{i}.arg;
        arg{1} = xL;
        U = real(feval(p.evalMap{i}.fcn,arg{1:end-1}));
        if isnan(U)            
            U = p.evalMap{i}.properties.range(2);            
        end
        arg{1} = xU;
        L = real(feval(p.evalMap{i}.fcn,arg{1:end-1}));
        if isnan(L)            
            L = p.evalMap{i}.properties.range(1);                        
        end
    elseif isequal(p.evalMap{i}.properties.convexity,'convex') && ~isempty(p.evalMap{i}.properties.stationary)
        L = p.evalMap{i}.properties.stationary(2);
        arg = p.evalMap{i}.arg;
        arg{1} = xL;
        U1 = real(feval(p.evalMap{i}.fcn,arg{1:end-1}));
        arg{1} = xU;
        U2 = real(feval(p.evalMap{i}.fcn,arg{1:end-1}));
        U = max(U1,U2);
    elseif isequal(p.evalMap{i}.properties.convexity,'concave') && ~isempty(p.evalMap{i}.properties.stationary)
        U = p.evalMap{i}.properties.stationary(2);
        arg = p.evalMap{i}.arg;
        arg{1} = xL;
        L1 = real(feval(p.evalMap{i}.fcn,arg{1:end-1}));
        arg{1} = xU;
        L2 = real(feval(p.evalMap{i}.fcn,arg{1:end-1}));
        L = min(L1,L2);
    else
        % To get some kind of bounds, we just sample the function
        % and pick the min and max from there. This only works for
        % simple functions with limited variation...
        % We assume it is f(x,parameters)
        if length(xL)>1
            % We can only sample if it is a scalar function
            disp([p.evalMap{i}.fcn ' is not supported in the global solver (only scalar functions support)'])
            error([p.evalMap{i}.fcn ' is not supported in the global solver'])
        end
        if ~isinf(xL) & ~isinf(xU)
            xtest = linspace(xL,xU,100);
            arg = p.evalMap{i}.arg;
            arg{1} = xtest;
            values = real(feval(p.evalMap{i}.fcn,arg{1:end-1}));
            [minval,minpos] = min(values);
            [maxval,maxpos] = max(values);
            xtestmin = linspace(xtest(max([1 minpos-5])),xtest(min([100 minpos+5])),100);
            xtestmax = linspace(xtest(max([1 maxpos-5])),xtest(min([100 maxpos+5])),100);
            arg{1} = xtestmin;
            values1 = real(feval(p.evalMap{i}.fcn,arg{1:end-1}));
            arg{1} = xtestmax;
            values2 = real(feval(p.evalMap{i}.fcn,arg{1:end-1}));
            L = min([values1 values2]);
            U = max([values1 values2]);
            if ~isempty(p.evalMap{i}.properties.singularity)
                if xL <= p.evalMap{i}.properties.singularity && xU >= p.evalMap{i}.properties.singularity
                    arg = p.evalMap{i}.arg;                                    
                    arg{1} = p.evalMap{i}.properties.singularity-1/inf;
                    values = real(feval(p.evalMap{i}.fcn,arg{1:end-1}));
                    L = min(L,values);
                    U = max(U,values);
                    arg{1} = p.evalMap{i}.properties.singularity+1/inf;
                    values = real(feval(p.evalMap{i}.fcn,arg{1:end-1}));
                    L = min(L,values);
                    U = max(U,values);
                end
            end
        end
    end
end
p.lb(p.evalVariables(i)) = max([p.lb(p.evalVariables(i)) L],[],2);
p.ub(p.evalVariables(i)) = min([p.ub(p.evalVariables(i)) U],[],2);