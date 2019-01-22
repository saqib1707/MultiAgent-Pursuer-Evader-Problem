function [ceq] = non_linear_equality(x, vemax, vemax2, Ne, Ni, var, K, ip, ti)
    count = 0;

    centroid = zeros(2,Ni);
    for t = 0:Ni-1
        if t == 0
            centroid(:,t+1) = sum(reshape(ip(1:Ne*2),2,Ne),2);
        else
            centroid(:,t+1) = sum(reshape(x(var*(t-1)+1:var*(t-1)+Ne*2),2,Ne),2);
        end
    end

    for i = 1:Ne
        for t = 0:Ni-1
            if t == 0
                losv = ip(2*i-1:2*i) - ip(var-1:var);
                term1 = (x(2*i-1:2*i) - ip(2*i-1:2*i))/ti;
                term3 = x(var-1:var) - ip(var-1:var);
                nn_centroid = (centroid(:,t+1) - ip(2*i-1:2*i))/(Ne-1);
                sec_direction = nn_centroid - ip(2*i-1:2*i);
            else
                losv = x(var*(t-1)+2*i-1:var*(t-1)+2*i) - x(var*(t-1)+var-1:var*(t-1)+var);
                term1 = (x(var*t+2*i-1:var*t+2*i) - x(var*(t-1)+2*i-1:var*(t-1)+2*i))/ti;
                term3 = x(var*t+var-1:var*t+var) - x(var*(t-1)+var-1:var*(t-1)+var);
                nn_centroid = (centroid(:,t+1) - x(var*(t-1)+2*i-1:var*(t-1)+2*i))/(Ne-1);
                sec_direction = nn_centroid - x(var*(t-1)+2*i-1:var*(t-1)+2*i);
            end

            sec_direction = sec_direction/norm(sec_direction);
            distance = norm(losv);
            term2 = exp(-K*distance);
            costheta = dot(losv,term3)/(norm(term3)*distance);

            for z=1:2
                count = count + 1;
                ceq(count) = term1(z) - vemax*0.5*term2*(1+costheta)*losv(z)/distance - vemax2*term2*sec_direction(z);
            end
        end
    end
end