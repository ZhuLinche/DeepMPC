classdef MPCController < AbstractController
    % Class representing on-line MPC controllers
    %
    % Constructor:
    %   ctrl = MPCController(model, N)
    %         model: prediction model
    %             N: prediction horizon

    methods
		
		function out = getName(obj)
			out = 'MPC controller';
		end
		
		function out = isExplicit(obj)
			out = false;
		end

        function obj = MPCController(varargin)
            % Constructor:
            %   ctrl = Controller(model, N)
			%         model: prediction model
			%             N: prediction horizon

			%obj.addlistener('model', 'PreGet', @obj.modelPreGetEvent);
			%obj.addlistener('model', 'PostGet', @obj.modelPostGetEvent);
			%obj.addlistener('model', 'PreSet', @obj.modelPreSetEvent); 
			%obj.addlistener('model', 'PostSet', @obj.modelPostSetEvent); 
			obj.addlistener('N', 'PostSet', @obj.NPostSetEvent);

			if nargin==0
				return
			end

			obj.importUserData(varargin{:});
			
			% Let's not construct the optimizer here, since it is expected
			% that users will modify the controller often.
			% 			if ~isempty(obj.N)
			% 				% If prediction horizon was provided, construct the
			% 				% optimizer immediately. Otherwise it will be constructed
			% 				% once evaluate() is called for the first time (also
			% 				% applies if the prediction horizon is changed afterwards).
			% 				obj.construct();
			% 			end
		end
		
        function EMPC = toExplicit(obj)
            % Computes the explicit controller
            
			% make sure we have the prediction horizon available
			error(obj.assert_controllerparams_defined);
			
            EMPC = EMPCController;
			EMPC.model = obj.model;
			EMPC.N = obj.N;
			
			% propagate custom YALMIP setup if provided
			if ~isempty(obj.yalmipData)
				% use YALMIP data to construct the explicit solution
				EMPC.fromYALMIP(obj.yalmipData);
			else
				% construct the explicit solution
				EMPC.construct();
			end
		end
        		
		function obj = construct(obj, sdpopt)
			% Converts the MPC problem into a YALMIP's optimizer object
            %
            % Syntax:
            %   ctrl.construct()
            %   ctrl.construct(sdpopt) -- uses the specified sdpsettings

			% make sure we have the prediction horizon available
			error(obj.assert_controllerparams_defined);
			
			if isempty(obj.yalmipData)
				Y = obj.toYALMIP();
			else
				Y = obj.yalmipData;
            end
			
            if nargin==2
                Y.internal.sdpsettings = sdpopt;
            end
			% construct YALMIP's optimizer object with the first state as
			% the initial condition
			obj.optimizer = optimizer(Y.constraints, Y.objective, ...
				Y.internal.sdpsettings, ...
				Y.internal.parameters, Y.internal.requested);
			
			obj.markAsUnmodified();
		end
		
        function [u, feasible, openloop] = evaluate(obj, xinit, varargin)
            % Solves the MPC optimization problem for a given initial
            % condition
			%
			% u = controller.evaluate(x0) solves the MPC optimization
			% problem using x0 as the initial condition and returns the
			% first element of the optimal sequence of control inputs. If
			% the problem is infeasible, "u" will be NaN.
			%
			% [u, feasible] = controller.evaluate(x0) also returns a
			% boolean flag indicating whether the optimization problem had
			% a feasible solution.
			%
			% [u, feasible, openloop] = controller.evaluate(x0) also
			% returns the open-loop predictions of states, inputs and
			% outputs in "openloop.X", "openloop.Y", and "openloop.Y",
			% respectively. Value of the optimal cost is returned in
			% openloop.cost. 
			%
			% u = controller.evaluate(x0, 'x.reference', ref, 'u.prev', u0)
			% includes "ref" and "u0" into the vector of initial
			% conditions.

			% make sure we have the prediction horizon available
			error(obj.assert_controllerparams_defined);
			
			error(validate_vector(xinit, obj.nx, 'initial state'));
			
			if obj.wasModified() || isempty(obj.optimizer)
				% Re-generate the optimizer if the problem setup was
				% changed
				obj.construct();
			end

			% assemble the vector of initial conditions. Include any
			% variables that were declared by filters as those which need
			% proper initialization.
			xinit = obj.parse_xinit(xinit, varargin{:});
			
			% use a pre-constructed optimizer object for faster
			% evaluation
			[U, status] = obj.optimizer{xinit};
            % these statuses indicate a feasible solution 
            % (from "help yalmiperror"):
            %  0: Successfully solved
            %  3: Maximum #iterations or time-limit exceeded
            %  4: Numerical problems
            %  5: Lack of progress
            feasible = ismember(status, [0, 3, 4, 5]);
			if ~feasible
				J = Inf;
				u = NaN(obj.nu, 1);
				U = NaN(size(U));
			elseif isempty(U)
				error('Ooops, something went wrong. Please report to mpt@control.ee.ethz.ch');
			else
				J = U(1);
				u = U(2:obj.nu+1);
			end

			if nargout==3
				% also return the open-loop profiles
				%
				% TODO: also return the 'd' and 'z' variables
				openloop.cost = J;
				U = U(2:end);
				openloop.U = reshape(U(1:obj.nu*obj.N), [obj.nu obj.N]);
				U = U(obj.nu*obj.N+1:end);
				openloop.X = reshape(U(1:obj.nx*(obj.N+1)), [obj.nx, obj.N+1]);
				U = U(obj.nx*(obj.N+1)+1:end);
				openloop.Y = reshape(U(1:obj.model.ny*obj.N), [obj.model.ny, obj.N]);
			end
		end
		
		function new = saveobj(obj)
			% save method

			new = saveobj@AbstractController(obj);

			% YALMIP's @optimizer objects cannot be saved
			new.optimizer = [];
			
        end
        
        function Matrices = getMatrices(obj)
            % Returns matrices of the parametric problem
            %
            %   Matrices = ctrl.getMatrices()
            %
            % returns the matrices of the pQP/pLP formulation of the MPC
            % optimization problem. If Matrices.qp=1, the problem is
            % formulated as
            %   min_U 1/2 U'*H*U + x'*F*U + Cf*U + x'*Y*x + Cx*x + Cc
            %    s.t. G*U <= W + E*x
            %
            % If the problem is a pLP (identified by Matrices.qp=0), then:
            %   min_U H*U + Cx*x + Cc
            %    s.t. G*U <= W + E*x
            %
            % Here, U is the open-loop sequence of optimal control inputs
            % and "x" is the vector of parameters.
            
            % make sure we have the prediction horizon available
			error(obj.assert_controllerparams_defined);
            
            % export the problem to YALMIP
            Ydata = obj.toYALMIP();
            
            Matrices = mpt_yalmip2mpt(Ydata.constraints, Ydata.objective, ...
                Ydata.internal.parameters(:), Ydata.variables.u(:));
        end
        
        function f = toFiordos(obj)
            % Exports the MPC controller to FiOrdOs
            %
            %   ctrl.toFiordos()
            
            Y = obj.toYALMIP();
            opt = optimizer(Y.constraints, Y.objective, [], ...
                Y.internal.parameters, Y.variables.u(:));
            f = fiordos(opt);
        end
	end

	methods(Access=protected)
		
		function status = wasModified(obj)
			% Returns true if either the prediction horizon or the problem
			% properties (e.g. constraints) were changed by the user
			
			status = obj.wasModified@AbstractController || ...
				any(cellfun(@any, obj.model.forEachComponent('all', 'wasModified')));
		end
		
		function obj = markAsUnmodified(obj)
			% Marks the object as unmodified
			
			obj.markAsUnmodified@AbstractController;
			obj.model.forEachComponent('all', 'markAsUnmodified');
		end

	end
	
	methods(Access=protected, Hidden)
		
		
		function modelPostSetEvent(obj, source, event)
		end

		function modelPreSetEvent(obj, source, event)
		end

		function modelPreGetEvent(obj, source, event)
		end

		function modelPostGetEvent(obj, source, event)
		end
		
		function NPreSetEvent(obj, source, event)
		end

		function NPostSetEvent(obj, source, event)
			% triggered after the prediction horizon was changed
			
			obj.markAsModified();
		end

	end

	methods(Static, Hidden)
		
		function checkOptimizer(optimizer)
			% validate that the optimizer is either [] or an instance of
			% the @optimizer class
			
			if ~((isa(optimizer, 'double') && isempty(optimizer)) || ...
					isa(optimizer, 'optimizer'))
				error('The optimizer must be an instance of the @optimizer class.');
			end
		end
	end
end
