function [ceq] = non_linear_equality_1(x, vemax, Ne, Ni, var, K, ip, ti)
    count = 0;
    for i = 1:Ne
        for t = 0:Ni-1
            if t == 0
                losv = ip(2*i-1:2*i) - ip(Ne*2+1:Ne*2+2);
                term1 = (x(2*i-1:2*i) - ip(2*i-1:2*i))/ti;
                term3 = x(Ne*2+1:Ne*2+2) - ip(Ne*2+1:Ne*2+2);
            else
                losv = x(var*(t-1)+2*i-1:var*(t-1)+2*i) - x(var*(t-1)+Ne*2+1:var*(t-1)+Ne*2+2);
                term1 = x(var*t+2*i-1:var*t+2*i) - x(var*(t-1)+2*i-1:var*(t-1)+2*i);
                term3 = x(t*var+Ne*2+1:t*var+Ne*2+2) - x(var*(t-1)+Ne*2+1:var*(t-1)+Ne*2+2);
            end
            
            distance = norm(losv);
            term2 = exp(-K*distance);
            costheta = (transpose(losv)*term3)/(norm(term3)*distance);

            for z=1:2
                count = count + 1;
                ceq(count) = term1(z) - vemax*0.5*term2*(1+costheta)*losv(z)/distance;
            end
        end
    end
end