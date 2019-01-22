function [cieq] = non_linear_inequality(x, var, Ni, Ne, ti, ip, vpmax, epsilon)
    centroid = [0;0];
    for i=1:2:Ne*2
        centroid = centroid + x((Ni-1)*var+i:(Ni-1)*var+i+1);
    end
    centroid = centroid/Ne;
    
    count = 0;
    for i=1:2:Ne*2
        count = count + 1;
        cieq(count) = norm(x((Ni-1)*var+i:(Ni-1)*var+i+1) - centroid) - epsilon;
    end
    
    count = count + 1;
    cieq(count) = norm(x(var-1:var) - ip(var-1:var)) - vpmax*ti;
    for t=2:Ni
        count = count + 1;
        cieq(count) = norm(x(var*t-1:var*t) - x(var*(t-1)-1:var*(t-1))) - vpmax*ti;
    end
end