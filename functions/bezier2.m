function fcn = bezier2(coeff,ss,degree)
    if nargin < 3
        degree = 0;
    end
    
    fcn = zeros(size(ss));
    if degree == 0
        % 0th order derivative (none)
        for i = 1:length(ss)
            s = ss(i);
            m = length(coeff) - 1;

            for k = 0:m
                fcn(i) = fcn(i) + coeff(k+1)*singleterm_bezier(m,k,s);
            end
        end
    elseif degree == 1
        % 1st order derivative
        dcoeff = diff_coeff(coeff);
        fcn = bezier2(dcoeff,ss,0);
    elseif degree == 2
        % 2nd order derivative
        dcoeff = diff_coeff(coeff);
        d2coeff = diff_coeff(dcoeff);
        fcn = bezier(d2coeff,ss,0);
    else
        disp('no valid degree given')
        fcn = bezier2(coeff,ss,0);
    end
end

%% Helper functions
function be = singleterm_bezier(m,k,s)
    if k == 0
        be = nchoosek(m,k) * (1-s)^(m-k);
    elseif k == m
        be = nchoosek(m,k) * s^k;
    else
        be = nchoosek(m,k) * s^k * (1-s)^(m-k);
    end
end

function val = nchoosek(m,k)
    val = factorial(m) / (factorial(k)*factorial(m-k));
end
    
function dcoeff = diff_coeff(coeff)
    m = length(coeff) - 1;
    A = zeros(m,m+1);
    
    for i = 0:m-1
        A(i+1,i+1) = -(m-i)*nchoosek(m,i) / nchoosek(m-1,i);
        A(i+1,i+2) =  (i+1)*nchoosek(m,i+1) / nchoosek(m-1,i);
    end
    A(m,m+1) = m*nchoosek(m,m);
    dcoeff = A*coeff';
end





