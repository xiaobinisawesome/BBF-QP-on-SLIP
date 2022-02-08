function c = costFcnBezierCustom(bv,tdata,zdata,before,after)
    z = zeros(size(tdata));
    bv = [before bv after];
    for i = 1:length(tdata)
        z(i) = bezier2(bv,tdata(i));
    end
    c = rms(z-zdata);
end