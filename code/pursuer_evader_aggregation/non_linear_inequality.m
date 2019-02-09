function [cieq] = non_linear_inequality(x, var, Ni, Ne, ti, ip, vpmax, vpmin, epsilon, fd)
    parameters = horzcat(ip,reshape(x,var,Ni));
%     final_centroid = mean(reshape(parameters(1:Ne*2,Ni+1),2,Ne),2);
%     cieq(1:Ne,1) = sqrt(sum((reshape(parameters(1:Ne*2,Ni+1),2,Ne) - repmat(final_centroid,[1,Ne])).^2,1)) - epsilon;
    cieq(1:Ne,1) = sqrt(sum((reshape(parameters(1:Ne*2,Ni+1),2,Ne) - repmat(fd,[1,Ne])).^2,1)) - epsilon;
    cieq(Ne+1:Ne+Ni,1) = sqrt(sum((parameters(var-1:var,2:Ni+1) - parameters(var-1:var,1:Ni)).^2,1)) - vpmax*ti;
    cieq(Ne+Ni+1:Ne+2*Ni,1) = vpmin*ti - sqrt(sum((parameters(var-1:var,2:Ni+1) - parameters(var-1:var,1:Ni)).^2,1));
end