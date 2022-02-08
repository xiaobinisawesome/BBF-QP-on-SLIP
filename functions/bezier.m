function bi = bezier(degree,s,alpha)
% degree can only be 0,1 or 2.
% degree = 0 is the regular bi(s)
% degree = 1 is dbi(s)/ds
% degree = 2 is d^2bi(s)/ds^2
bi = 0;
k = 0;
M = length(alpha)-1;
if degree == 0
    while k<=M
        bi = bi+alpha(k+1)*(factorial(M)/(factorial(k)*factorial(M-k)))*...
            (s^k)*((1-s)^(M-k));
        k = k+1;
    end
elseif degree == 1
    while k<=M-1
        bi = bi+(alpha(k+2)-alpha(k+1))*...
        (factorial(M)/(factorial(k)*factorial(M-k-1)))*...
            (s^k)*((1-s)^(M-k-1));
        k = k+1;
    end
else
    while k<=M-2
        bi = bi+(alpha(k+3)-2*alpha(k+2)+alpha(k+1))*...
        (factorial(M-2)/(factorial(k)*factorial(M-k-2)))*...
            (s^k)*((1-s)^(M-k-2))*(M^2-M);
        k = k+1;
    end
end
    
    