function [ wraped ] = wrapN( index, length )
%WRAPN wrap the index around the length for matlab indexing
% convert the index to zero based (-1), then modulo (cause mod is zero
% based), then convert to one based index (+1) for matlab

wraped = mod(index - 1, length) + 1;


end

