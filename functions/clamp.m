function valOut = clamp(valIn,valMin,valMax)
    if valIn < valMin
        valOut = valMin;
    elseif valIn > valMax
        valOut = valMax;
    else
        valOut = valIn;
    end
end