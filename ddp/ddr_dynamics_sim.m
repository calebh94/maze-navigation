function x = ddr_dynamics_sim(xo,u_new,Horizon,dt,sigma, params)

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

for k = 1:(Horizon-1)   
    

    if u_new(1,1) == 0
        xdot(1,k+1) = r_rad*d_theta_nom * cos(x(3,k));
        xdot(2,k+1) = -r_rad*d_theta_nom * sin(x(3,k));
        xdot(3,k+1) = 0;
        
        x(1,k+1) = x(1,k) + dt.*xdot(1,k+1);
        x(2,k+1) = x(2,k) + dt.*xdot(2,k+1);
        x(3,k+1) = x(3,k) + dt.*xdot(3,k+1);
        
    else
        u_new(1,1) = ((max_u - min_u) / 2) * tanh(u_new(1,1)) + (max_u + min_u) / 2;
        d_theta_right = d_theta_nom + u_new(1,k) * d_theta_max_dev;
        d_theta_left = d_theta_nom - u_new(1,k) * d_theta_max_dev;

        R = r_rad * d_theta_right;
        L = l_rad * d_theta_left;

        xdot(1,k+1) = (wb/2) * ( (R+L)/(R-L) ) * ( sin( ((R-L)/wb) + x(3,k) ) - sin(x(3,k)) );
        xdot(2,k+1) = -(wb/2) * ( (R+L)/(R-L) ) * ( cos( ((R-L)/wb) + x(3,k) ) - cos(x(3,k)) );
        xdot(3,k+1) = (R-L) / wb;

        x(1,k+1) = x(1,k) + dt.*xdot(1,k+1);
        x(2,k+1) = x(2,k) + dt.*xdot(2,k+1);
        x(3,k+1) = x(3,k) + dt.*xdot(3,k+1);

end

end