function draw_circle(x_center ,y_center, r)
    % x and y are the coordinates of the center of the circle
    % r is the radius of the circle
    % 0.01 is the angle step, bigger values will draw the circle faster but
    % you might notice imperfections (not very smooth)

    ang=0:0.001:2*pi; 
    xp=r*cos(ang);
    yp=r*sin(ang);
    plot(x_center+xp, y_center+yp, 'color', 'black');
end