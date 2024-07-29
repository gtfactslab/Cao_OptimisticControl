function out = d_cos(w, what)
    if sin(w) <= 0 && sin(what) <= 0 && abs(w - what) <= pi
        out = cos(w);
    elseif sin(w) >= 0 && sin(what) >= 0 && abs(w - what) <= pi
        out = cos(what);
    elseif abs(w - what) >= 2*pi
        out = sign(w - what);
    elseif sin(w) >= 0 && 0 >= sin(what)
        out = sign(w - what);
    elseif sin(w)*sin(what) >= 0 && abs(w - what) >= pi
        out = sign(w - what);
    elseif w <= what && sin(w) <= 0 && sin(what) >= 0 && abs(w - what) <= 2*pi
        out = min([cos(w), cos(what)]);
    elseif w >= what && sin(w) <= 0 && sin(what) >= 0 && abs(w - what) <= 2*pi
        out = max([cos(w), cos(what)]);          
    end
end