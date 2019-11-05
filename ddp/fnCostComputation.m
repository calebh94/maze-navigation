function [Cost] =  fnCostComputation(x_traj,u_new,p_targets,dt,Q_f,R,Vx)


 [~,Horizon] = size(x_traj);
 Cost = 0;
  
 [~,w] = size(p_targets);
 
 p_target = p_targets(:,end);
 
% Q = Vx(:,j);

Q(1,1) = 100;
Q(2,2) = 100;
Q(3,3) = 0;
 
 for j =1:(Horizon-1)
     % Find closet target
%      for m=1:w
%          x_errors(:,m) = norm(abs(x_traj(1:2,j) - p_targets(1:2,m)));
%      end
    if w >= 3
        if j < (Horizon-1) * (1 / length(w))
            x_target = p_targets(:,1);
        elseif j >=  (Horizon-1) * (1 / length(w)) && j <  (Horizon-1) * (2 / length(w))
            x_target = p_targets(:,2);
        elseif j >=  (Horizon-1) * (2 / length(w)) && j < (Horizon-1)
            x_target = p_targets(:,3);
        else
            x_target = p_targets(:,3);
        end
    else
        x_target = p_target(:,1);
    end
    
%      [~, ind] = min(x_errors); 
%      x_target = p_targets(:,ind);
     x_error = x_traj(:,j) - x_target;
     
  
     
     tracking_cost = 0.5 * x_error' * Q * x_error;
     control_cost = Cost + 0.5 * u_new(:,j)' * R * u_new(:,j) * dt;
%     Cost = Cost + 0.5 * u_new(:,j)' * R * u_new(:,j) * dt + 0.5 * x_traj(:,j)' * Q * x_traj(:,j);
     Cost = Cost + tracking_cost + control_cost;
 end
 
 TerminalCost= 1/2*(x_traj(:,Horizon) - p_target)'*Q_f * (x_traj(:,Horizon) - p_target);
 
 Cost = Cost + TerminalCost;