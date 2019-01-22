function [] = evader_path_verify(Ne, ip, Ni, ti, opt_x, vemax, vemax2, K, var, pursuer_x, pursuer_y, epsilon)

    % Below part is just for verifying whether evaders and pursuer
    % is following the control law
    % Reconstructing the path of the evader based on the pursuer path obtained
    % from fmincon

    centroid = zeros(2,Ni);
    ep = zeros(2*Ne, Ni+1);
    ep(:,1) = ip(1:Ne*2);
    centroid(:,1) = sum(reshape(ep(:,1),2,Ne),2);

    for i = 1:Ne
        temp = (centroid(:,1) - ep(2*i-1:2*i,1))/(Ne-1) - ep(2*i-1:2*i,1);
        temp = temp/norm(temp);
        ep(2*i-1:2*i,2) = compute_enc(ep(2*i-1:2*i,1), ip(var-1:var), opt_x(var-1:var), vemax, vemax2, ti, K, temp);
    end

    for t=2:Ni
        centroid(:,t) = sum(reshape(ep(:,t),2,Ne),2);
        for i=1:Ne
            temp = (centroid(:,t) - ep(2*i-1:2*i,t))/(Ne-1) - ep(2*i-1:2*i,t);
            temp = temp/norm(temp);
            ep(2*i-1:2*i,t+1) = compute_enc(ep(2*i-1:2*i,t), opt_x(var*(t-1)-1:var*(t-1)), opt_x(var*t-1:var*t), vemax, vemax2, ti, K, temp);
        end
    end

    epgt = [ip; opt_x];
    epgt = reshape(epgt, var, Ni+1);
    epgt = epgt(1:Ne*2,:);

    diff = ep-epgt;

    final_centroid = sum(reshape(ep(:,Ni+1),2,Ne),2)/Ne;

    figure(2);
    for i=1:Ne
        plot(ep(2*i-1,1),ep(2*i,1), 'o-', 'color', 'red');hold on;
        plot(ep(2*i-1,2:Ni),ep(2*i,2:Ni), 'o-', 'color', 'magenta');hold on;
        plot(ep(2*i-1,Ni+1),ep(2*i,Ni+1), 'o-', 'color', 'green');hold on;
    end
    plot(pursuer_x, pursuer_y, 'o-', 'color', 'blue');hold on;
    draw_circle(final_centroid(1), final_centroid(2), epsilon);
    grid on;
    xlabel('X');
    ylabel('Y');
    title('pursuer-evader-aggregation-gt-check');
    hold off;
end