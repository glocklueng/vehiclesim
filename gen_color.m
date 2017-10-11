function [ color ] = gen_color( state, param, max_val )
%GEN_COLOR generates a color based on the state and which parameter to use

%default black
color = [0, 0, 0];

%color based on breaking / acceleration
val = state.(param);

if val > 0 %green
    color(2) = min(val / max_val, 1.0);
elseif val < 0 %red
    color(1) = min(-val / max_val, 1.0);
end

end

