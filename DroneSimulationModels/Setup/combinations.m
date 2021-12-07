%This function generates all possible combinations of a input vector for
%the polyhedron function to use.

function verticies = combinations(Xlimit)
    t = length(Xlimit);
    x = nchoosek(repmat([1 -1],1,t),t);
    perm = unique(x, 'rows');
    verticies = perm.*Xlimit;
end
