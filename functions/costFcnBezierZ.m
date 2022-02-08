function c = costFcnBezierZ(bv,tdata1,tdata2,tdata3,zdata1,zdata2,zdata3,nS,nDst,nDsw)

bvSSP = bv(1:nS);
bvDSPst = bv(nS+1:nS+nDst);
bvDSPsw = bv(nS+nDst+1:nS+nDst+nDsw);
c = 0;

z = zeros(size(tdata1));
for i = 1:length(tdata1)
    z(i) = bezier2(bvSSP,tdata1(i));
end
c = c + rms(z-zdata1);

z = zeros(size(tdata2));
for i = 1:length(tdata2)
    z(i) = bezier2(bvDSPst,tdata2(i));
end
c = c + rms(z-zdata2);

z = zeros(size(tdata3));
for i = 1:length(tdata3)
    z(i) = bezier2(bvDSPsw,tdata3(i));
end
c = c + rms(z-zdata3);
end