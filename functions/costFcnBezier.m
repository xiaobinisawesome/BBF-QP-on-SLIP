function c = costFcnBezier(bv,tdata,zdata)
    z = zeros(size(tdata));
    for i = 1:length(tdata)
        z(i) = bezier2(bv,tdata(i));
    end
    c = rms(z-zdata);
end