function xdot = ddr_dyn(xo,u_new, params)

l = params.length;
w = params.width;
wb = params.wb;
l_rad = params.l_radius;
r_rad = params.r_radius;
d_theta_nom = params.d_theta_nom;
d_theta_max_dev = params.d_theta_max_dev;
d_theta_reverse = params.d_theta_reverse;

x = xo;

max_u = 1;
min_u = -1;

if u_new(1,1) == 0
    xdot(1,1) = r_rad*d_theta_nom * cos(x(3,1));
    xdot(2,1) = -r_rad*d_theta_nom * sin(x(3,1));
    xdot(3,1) = 0;
    
else
    u_new(1,1) = ((max_u - min_u) / 2) * tanh(u_new(1,1)) + (max_u + min_u) / 2;
    d_theta_right = d_theta_nom + u_new(1,1) * d_theta_max_dev;
    d_theta_left = d_theta_nom - u_new(1,1) * d_theta_max_dev;

    R = r_rad * d_theta_right;
    L = l_rad * d_theta_left;

    xdot(1,1) = (wb/2) * ( (R+L)/(R-L) ) * ( sin( ((R-L)/wb) + x(3,1) ) - sin(x(3,1)) );
    xdot(2,1) = -(wb/2) * ( (R+L)/(R-L) ) * ( cos( ((R-L)/wb) + x(3,1) ) - cos(x(3,1)) );
    xdot(3,1) = (R-L) / wb;
end