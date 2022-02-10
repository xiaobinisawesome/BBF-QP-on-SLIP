function c = costFcnBezierF(bv,tdataSSP,tdataDSPst,tdataDSPsw,zdataSSP,zdataDSPst,zdataDSPsw,nS,nDst,nDsw)

bvSSP = bv(1:nS);
bvDSPst = bv(nS+1:nS+nDst);
bvDSPsw = bv(nS+nDst+1:nS+nDst+nDsw);
c = 0;

z = zeros(size(tdataSSP));
for i = 1:length(tdataSSP)
    z(i) = bezier2(bvSSP,tdataSSP(i));
end
c = c + rms(z-zdataSSP);

z = zeros(size(tdataDSPst));
for i = 1:length(tdataDSPst)
    z(i) = bezier2(bvDSPst,tdataDSPst(i));
end
c = c + rms(z-zdataDSPst);

z = zeros(size(tdataDSPsw));
for i = 1:length(tdataDSPsw)
    z(i) = bezier2(bvDSPsw,tdataDSPsw(i));
end
c = c + rms(z-zdataDSPsw);
end