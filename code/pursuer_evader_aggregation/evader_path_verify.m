function [enc] = evader_path_verify(ecc, pcc, pnc, vemax, ti, K)
    losv = ecc - pcc;
    d = norm(losv);
    term1 = pnc - pcc;
    costheta = dot(losv, term1)/(d*norm(term1));
    enc = ecc + vemax*0.5*ti*exp(-K*d)*(1+costheta)*losv/d;
end