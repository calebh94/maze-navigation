function [x_traj, u_traj, cost_traj] = ddp_ddr(params, state, goals, init)
% ddp:  Differential Dynamic Programming for Differential Drive Robot
% ACRL Homework 3 - Spring 2019 - Caleb Harris

l = params.length;
w = params.width;
wb = params.wb;
l_rad = params.l_radius;
r_rad = params.r_radius;
d_theta_nom = params.d_theta_nom;
d_theta_max_dev = params.d_theta_max_dev;
d_theta_reverse = params.d_theta_reverse;

Horizon = params.horizon;
num_iter = params.iterations;
dt = params.dt;
Q_f = params.Q_f;
R = params.R;
gamma = params.gamma;

x0 = [state.x, state.y, state.theta]';
p_targets = goals;
p_target = goals(:,end);

% Initial control:
u_k = zeros(1,Horizon-1);
% u_k = init.u_k;

du_k = zeros(1,Horizon-1);
% du_k = init.u_k;

% Initial trajectory:
x_traj = zeros(3,Horizon);
% x_traj = init.x_traj;

avg_cost = 0;
for k = 1:num_iter
    
    for j = 1:(Horizon-1)

        [l0,l_x,l_xx,l_u,l_uu,l_ux, l_xu] = fnCost_ddr(x_traj(:,j), u_k(:,j), j, R, dt); %Evaluate Learning cost function

        L0(j) = l0*dt;
        Lx(:,j) = l_x'*dt;
        Lu(:,j) = l_u'*dt;
        Lxx(:,:,j) = l_xx*dt;      
        Luu(:,:,j) = l_uu*dt;
        Lux(:,:,j) = l_ux*dt; 
        Lxu(:,:,j) = l_xu*dt;

        [dfx, dfu] = finite_difference_fcn(x_traj(:,j),u_k(:,j),3,1, params);
        A(:,:,j) = eye(3,3) + dfx * dt;
        B(:,:,j) = dfu * dt;  
        
    end
    
    % Estimate Value Function at terminal time
    Vxx(:,:,Horizon)= Q_f;
%     display(size(p_target))
%     display(p_target)
    Vx(:,Horizon) = Q_f * (x_traj(:,Horizon) - p_target); 
    V(Horizon) = 0.5 * (x_traj(:,Horizon) - p_target)' * Q_f * (x_traj(:,Horizon) - p_target); 

    
    %------------------------------------------------> Backpropagation of the Value Function
    for j = (Horizon-1):-1:1
        
        Q0 = L0(j) + V(j+1);
        Qx = Lx(:,j) + A(:,:,j)'*Vx(:,j+1);
        Qu = Lu(:,j) + B(:,:,j)'*Vx(:,j+1);
        Qxx = Lxx(:,:,j) + A(:,:,j)'*Vxx(:,:,j+1)*A(:,:,j);
        Qxu = Lxu(:,:,j) + A(:,:,j)'*Vxx(:,:,j+1)*B(:,:,j);
        Qux = Lux(:,:,j) + B(:,:,j)'*Vxx(:,:,j+1)*A(:,:,j);
        Quu = Luu(:,:,j) + B(:,:,j)'*Vxx(:,:,j+1)*B(:,:,j);


        inv_Quu = inv(Quu);
        L_k(:,:,j)= - inv_Quu * Qux;
        l_k (:,j) = - inv_Quu * Qu; 

        V(:,j) = Q0 + Qu'*l_k(:,j) + 0.5*l_k(:,1)'*Quu*l_k(:,j);
        Vx(:,j) = Qx + L_k(:,:,j)'*Qu + Qxu*l_k(:,j) + L_k(:,:,j)'*Quu*l_k(:,j);
        Vxx(:,:,j) = Qxx + Qxu*L_k(:,:,j) + L_k(:,:,j)'*Qux + L_k(:,:,j)'*Quu*L_k(:,:,j);
    end
    
    %----------------------------------------------> Find the controls

    dx = zeros(3,1);
    for i=1:(Horizon-1)    
        du(:,i) = l_k(:,i) + L_k(:,:,i) * dx(:,i);
        dx(:,i+1) = A(:,:,i) * dx(:,i) + B(:,:,i).* du(:,i);  
        u_new(:,i) = u_k(:,i) + gamma * du(:,i); %Control input
        % Bound controls at [-1,1]
%         if u_new(:,i) > 1
%             u_new(:,i) = 1;
%         elseif u_new(:,i) < -1
%             u_new(:,i) = -1;
%         end
    end 
    max_u = 1;
    min_u = -1;
    u_k = ((max_u - min_u) / 2) * tanh(u_new) + (max_u + min_u) / 2;
%     u_k = tanh(u_new);
    
    %---------------------------------------------> Simulation of the Nonlinear System
    [x_traj] = ddr_dynamics_sim(x0,u_new,Horizon,dt,0, params); %Propagate nonlineaer dynamics
    [Cost(:,k)] =  fnCostComputation(x_traj,u_k,p_targets,dt,Q_f,R,Vx);
    x1(k,:) = x_traj(1,:);
   
%     fprintf('iLQG Iteration %d,  Current Cost = %e \n',k,Cost(1,k));
    
    avg_cost_new = mean(Cost(1,:));
    
    if abs(avg_cost - avg_cost_new) / avg_cost < 0.001 || avg_cost_new > avg_cost*1.01
        if avg_cost ~= 0
            break
        end
    else
        avg_cost = avg_cost_new;
    end
       
end
    
% fprintf('Solution found!\n')
x_traj = x_traj;
u_traj = u_k;
cost_traj = Cost;


end

