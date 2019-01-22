function [enc] = compute_enc(ecc, pcc, pnc, vemax, vemax2, ti, K, sec_direction)
    losv = ecc - pcc;
    d = norm(losv);
    term1 = pnc - pcc;
    costheta = dot(losv,term1)/(d*norm(term1));
    enc = ecc + vemax*0.5*ti*exp(-K*d)*(1+costheta)*losv/d + vemax2*ti*exp(-K*d)*sec_direction;
end