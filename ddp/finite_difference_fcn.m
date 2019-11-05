function [f_x f_u] = finite_difference_fcn(x,u,n,m, params)

f_x = zeros(n,n);
f_u = zeros(n,m);
h = 1.0; %step for finite differencing

x_k = x(:,1); %is nx1 vector
u_k = u(:,1);

f_k = ddr_dyn(x_k,u_k, params);
for j=1:n
    x_kplus1 = x_k;
    x_kplus1(j) = x_kplus1(j) + h;

    f_kplus1 = ddr_dyn(x_kplus1,u_k, params);

    f_x(:,j) = (f_kplus1 - f_k)/h;
end
for j=1:m
    u_kplus1 = u_k;
    u_kplus1(j) = u_kplus1(j) + h;

    f_kplus1 = ddr_dyn(x_k,u_kplus1, params);
    f_u(:,j) = (f_kplus1 - f_k)/h;
end

end