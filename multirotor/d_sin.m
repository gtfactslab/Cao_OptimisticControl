function out = d_sin(w, what)
    if cos(w) >= 0 && cos(what) >= 0 && abs(w - what) <= pi
        out = sin(w);
    elseif cos(w) <= 0 && cos(what) <= 0 && abs(w - what) <= pi
        out = sin(what);
    elseif abs(w - what) >= 2*pi
        out = sign(w - what);
    elseif cos(w) <= 0 && 0 <= cos(what)
        out = sign(w - what);
    elseif cos(w)*cos(what) >= 0 && abs(w - what) >= pi
        out = sign(w - what);
    elseif w <= what && cos(w) >= 0 && cos(what) <= 0 && abs(w - what) <= 2*pi
        out = min([sin(w), sin(what)]);
    elseif w >= what && cos(w) >= 0 && cos(what) <= 0 && abs(w - what) <= 2*pi
        out = max([sin(w), sin(what)]);          
    end
end