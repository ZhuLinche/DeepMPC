function filter = filter_setConstraint(obj)
% Polyhedral set constraint

% set up the filter
filter = FilterSetup;
filter.addField('value', [], @validate_polyhedron);

% the filter impacts the following calls:
filter.callback('constraints') = @on_constraints;
filter.callback('set') = @on_set;

end

%------------------------------------------------
function out = on_constraints(obj, varargin)
% called when constructing constraints

P = obj.setConstraint;
H = P.A;
K = P.b;
Heq = P.Ae;
Keq = P.be;

out = [];
for k = 1:obj.N
    out = out + [ H*obj.var(:, k) <= K ];
    if ~isempty(Heq)
        out = out + [ Heq*obj.var(:, k) == Keq ];
    end
end

end

%------------------------------------------------
function obj = on_set(obj, P)
% called before the set is changed

if isempty(P)
    return
elseif ~isa(P, 'Polyhedron')
	error('The input must be a polyhedron.');
elseif numel(P)~=1
	error('The input must be a single polyhedron.');
elseif P.Dim ~= obj.n
	error('The polyhedron must be in dimension %d.', obj.n);
elseif P.isEmptySet()
	error('The polyhedron must not be empty.');
end

% we require the H-representation, which we also normalize to avoid
% numerical problems
Q = Polyhedron(P); % make a coppy
for i = 1:numel(Q)
	Q(i).minHRep().normalize();
end

obj.setConstraint = Q;

end
