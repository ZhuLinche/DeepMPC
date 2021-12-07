function [A, B, Aeq, Beq, ind_eq] = mpt_ineq2eq(A, B, tol)
%MPT_INEQ2EQ Detects inequality constraints whihc form equalities
%
% [Ain, Bin, Aeq, Beq, ind_eq] = mpt_ineq2eq(A, B, )
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% For a system of inequalities A*x<=B, this function detects and returns those
% inequality constraints which form equalities. For instance:
%
% A = [1; -1; 1]; B = [1; -1; 2];
%
% The output will lead:
% Ain = [-1]; Bin = [2]; Aeq = [1]; Beq = 1;
%
% such that the original problem can be rewritten as:
%   Ain*x <= Bin
%   Aeq*x  = Beq
%
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
% A, B        - system of inequalities
%
% ---------------------------------------------------------------------------
% OUTPUT                                                                                                    
% ---------------------------------------------------------------------------
% Ain,Bin     - new system of inequalities
% Aeq,Beq     - system of equalities
% ind_eq      - indicies of equality constraints. ind_eq(i, :)=[k m] indicates
%               that inequalities A(k, :)-B(k) and A(m, :)-B(m) form one
%               equality constraint
%

% Copyright is with the following author(s):
%
% (C) 2006 Johan Loefberg, Automatic Control Laboratory, ETH Zurich,
%          loefberg@control.ee.ethz.ch
% (C) 2005 Michal Kvasnica, Automatic Control Laboratory, ETH Zurich,
%          kvasnica@control.ee.ethz.ch

% ---------------------------------------------------------------------------
% Legal note:
%          This program is free software; you can redistribute it and/or
%          modify it under the terms of the GNU General Public
%          License as published by the Free Software Foundation; either
%          version 2.1 of the License, or (at your option) any later version.
%
%          This program is distributed in the hope that it will be useful,
%          but WITHOUT ANY WARRANTY; without even the implied warranty of
%          MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
%          General Public License for more details.
% 
%          You should have received a copy of the GNU General Public
%          License along with this library; if not, write to the 
%          Free Software Foundation, Inc., 
%          59 Temple Place, Suite 330, 
%          Boston, MA  02111-1307  USA
%
% ---------------------------------------------------------------------------
global MPTOPTIONS
if isempty(MPTOPTIONS)
    MPTOPTIONS = mptopt;
end

narginchk(2, 3);
if nargin<3    
    % set tolerance if not defined
    tol = MPTOPTIONS.abs_tol;
end

if isempty(A) || isempty(B)
    error('mpt_ineq2eq: Matrix A and b must not be empty.');
end
    

[ne, nx] = size(A);
Aeq = [];
Beq = [];
ind_eq = [];
sumM = A*randn(size(A,2),1) + B;
for ii = 1:ne-1,
    s = sumM(1);
    
    % get matrix which contains all rows starting from ii+1 
    sumM = sumM(2:end,:);
    
    % possible candidates are those rows whose sum is equal to the sum of the
    % original row
    possible_eq = find(abs(sumM + s) < tol);
    if isempty(possible_eq),
        continue
    end
    possible_eq = possible_eq + ii;
    b1 = B(ii);    
    a1 = A(ii, :);
    
    % now compare if the two inequalities (the second one with opposite
    % sign) are really equal (hence they form an equality constraint)
    for jj = possible_eq',
        % first compare the B part, this is very cheap
        if abs(b1 + B(jj)) < tol,
            % now compare the A parts as well
            if norm(a1 + A(jj, :), Inf) < tol,
                % jj-th inequality together with ii-th inequality forms an equality
                % constraint
                ind_eq = [ind_eq; ii jj];
                break
            end
        end
    end
end

if isempty(ind_eq),
    % no equality constraints
    return
else
    % indices of remaining constraints which are inequalities
    ind_ineq = setdiff(1:ne, ind_eq(:));
    Aeq = A(ind_eq(:,1), :);
    Beq = B(ind_eq(:,1));
    A = A(ind_ineq, :);
    B = B(ind_ineq);
end
