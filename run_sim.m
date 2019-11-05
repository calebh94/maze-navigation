% -----------------------------------------------------------------------

%*****************************
% Load map files and parameters
%*****************************

close all;
clear all;
clc;

% load map_1.mat;
% map_name = 1;
% 
% load map_2.mat;
% map_name = 2;
% 
load map_3.mat;
map_name = 3;

load_sim_params;
load_ddp_params;

addpath('ddp/');

% scale is used to blow up the environment for display purposes only. Set
% to whatever looks good on your screen
scale = 10;

% determines the size of the map and creates a meshgrid for display
% purposes
[N,M] = size(map_struct.seed_map);
[x,y] = meshgrid(1:N, 1:M);

DISPLAY_ON = 1; % 1 - turns display on, 0 - turns display off
DISPLAY_TYPE = 1; % 0 - displays map as dots, 1 - displays map as blocks

%% -----------------------------------------------------------------------

%*****************************
% Training/Learning Phase
%*****************************
%
% Here is where you perform any training or learning that your algorithm
% will require, such as solving an MDP, determining a policy or a value
% function, etc.
%
% NOTE: At this stage, you may access any variables provided in params or 
% map_struct EXCEPT:
%
%           map_struct.map_samples 
%
% This is your test data, don't touch them! You may however create your own 
% map_samples if you feel that this will be beneficial. You may use any of
% the functions provided (such as motion model) or create your own. Also
% note that your algorithm must be kept constant for any and all maps. You 
% cannot hand tweak parameters or settings in your solver for specific 
% maps.
%
% This code block is allowed to run as long as you'd like, provided that it
% finishes in time for you to submit the assignment!
%
% Example:
%
% myPolicy_struct = solve_mdp(params, map_struct, ...)
% ...

[bridge_cord_dim, bridge_num] = size(map_struct.bridge_locations);
%% -----------------------------------------------------------------------

%*****************************
% Run Sim
%*****************************
%
% Here is where you'll actually run the simulation over the set of sampled
% maps. At each iteration, you will decide which action to take based on
% the state of the car, the observed map, and any training/learning data 
% that you may have created. There is no time limit on run time however you 
% must be able to run your solution on all map samples in time to submit 
% the assignment.

% Loop through each map sample
for i = 1:length(map_struct.map_samples) 
    calc_ddp = 0;
    ddp_step = 1;
    action_list_ddp = zeros(params.horizon,1);
    pred_states = zeros(3,params.horizon);
    opt_route = zeros(2,2);
    init.u_k = zeros(1,params.horizon-1);
    init.x_traj = zeros(3,params.horizon);
    
    calc_ddp = 0;
    ddp_step = 1;
    action_list_ddp = zeros(params.horizon,1);
    pred_states = zeros(3,params.horizon);
    final_pos.x = 1; final_pos.y = 1;
    opt_route = zeros(2,2);
    current_target=2;
    init.u_k = zeros(1,params.horizon-1);
    init.x_traj = zeros(3,params.horizon);
    backup = 0;
    cnt = 1;
    at_goal = 0;
    if map_name == 2
        lookahead = 3;
        params.gamma=0.2;
        params.horizon = 20;
    else
        lookahead = 3;
    end
    recalculate = 0;
    
    % Init Data Storage
    Dataset.map = map_name;
    Dataset.sample = i;
    Dataset.path = [map_struct.start.x, map_struct.start.y, 0]';
    Dataset.time = 0;
    Dataset.success = 0;
    Dataset.processing_time = 0;
    tic

    % Initialize the starting car state and observed map
    % observed_map is set to seed map, and the bridge information will be
    % updated once the car is within params.observation_radius
    initialize_state;
    orig_map = observed_map;
    
%     state.x = 31;
%     state.y = 10;
%     state.theta = 0;
   
    % display the initial state
    if (DISPLAY_ON)
        display_environment;
    end
    counter = 0;
    
    r_top = 1.0;
    r_bot = -1.0;
    
    % set a differences parameter
    diff = -1;
    
    % loop until maxCount has been reached or goal is found
    while (state.moveCount < params.max_moveCount && flags ~= 2)
        
        counter = counter+1;
      
        %---------------------------------------
        %
        %*****************************
        % Decide Action
        %*****************************
        %
        % Here you execute your policy (which may include re-planning or
        % any technique you consider necessary):
        
        % if the differences between current and updated bridge state is <0
        % means that the newly spotted bridge state is 0, which decreases
        % the total sum of the bridge state; therefore, we need to update
        % the optimal route from A* for the new bridge state.
               
        if diff < 0 || recalculate ==1
%             opt_route = aStar(observed_map, state, goal, map_struct, 0.4);
            opt_route = aStar_fixed_new(observed_map, state, goal, map_struct);

            k = 0;
            current_target = 2;
            recalculate = 0;
            
%             opt_route = opt_route(1:2:end,:);
        end
        
        % Check if target is final point
        if current_target+2 >= length(opt_route)
            opt_route(end+1, :) = opt_route(end,:);
            at_goal = 1;
%             lookahead = current_target - length(opt_route) + 3;
        end
        
        % Check if path point is actually safe
%         if (observed_map(opt_route(current_target,1),opt_route(current_target,2)) == 0)
%             xx = opt_route(current_target,1);  yy = opt_route(current_target,2);
%             if observed_map(xx+1,yy+1) == 1
%                 opt_route(current_target,1) = xx+1;
%                 opt_route(current_target,2) = yy+1;
%             elseif observed_map(xx-1,yy+1) == 1
%                 opt_route(current_target,1) = xx-1;
%                 opt_route(current_target,2) = yy+1;
%             elseif observed_map(xx+1,yy-1) == 1
%                 opt_route(current_target,1) = xx+1;
%                 opt_route(current_target,2) = yy-1;
%             elseif observed_map(xx-1,yy-1) == 1
%                 opt_route(current_target,1) = xx-1;
%                 opt_route(current_target,2) = yy-1;
%             else
%                 current_target = current_target + 1;
%             end
%         end
        
        % Get error from target
        state_error.x = abs(state.x - opt_route(current_target,1));
        state_error.y = abs(state.y - opt_route(current_target,2));
        
        if sqrt(state_error.x.^2 + state_error.y.^2) < 0.1
            current_target = current_target + 1;
            calc_ddp = 0;
        end
        
        if (ddp_step >= length(action_list_ddp))
            calc_ddp = 0;
        end
        
        % Get direction to next target after current one
        rad_error = atan2(opt_route(current_target+1,2) - opt_route(current_target,2), opt_route(current_target+1,1) - opt_route(current_target,1));
%         display(rad_error);
        
        % Get distance to future points to see if closer
        for trgt=current_target:current_target+1
            state_error_future.x = abs(state.x - opt_route(trgt+1,1));
            state_error_future.y = abs(state.y - opt_route(trgt+1,2));

            if (sqrt(state_error_future.x^2 + state_error_future.y^2)) < sqrt(state_error.x^2 + state_error.y^2)
                current_target = trgt + 1;
            end
        end
        
        
        if current_target > 3
            state_error_past.x = abs(state.x - opt_route(current_target-3,1));
            state_error_past.y = abs(state.y - opt_route(current_target-3,2));
        else
            state_error_past.x = 100;
            state_error_past.y = 100;
        end
%         if (sqrt(state_error_past.x^2 + state_error_past.y^2)) < sqrt(state_error.x^2 + state_error.y^2)
%             current_target = current_target - 1;
%         end
        
        

            
        % Calculating DDP solution
        calc_ddp = 0;
        if calc_ddp == 0 && backup == 0
%             ddp_goal.x = opt_route(current_target,1);
%             ddp_goal.y = opt_route(current_target,2);
%             ddp_goal.theta = rad_error;
%             num_wpts = length(opt_route) - current_target;
            if at_goal == 1
                num_wpts = 1;
            else
                num_wpts = lookahead;
    %             params.horizon = ceil((sqrt(state_error_future.x^2 + state_error_future.y^2)) * 0.85);
                point_dis = sqrt(abs(state.x - opt_route(current_target-1+lookahead,1))^2 + abs(state.y - opt_route(current_target-1+lookahead,2))^2);
                params.horizon = ceil(point_dis * 0.90 * (20/3));
                params.iterations = ceil(point_dis * 0.85 * (200/3));
            end
            ddp_goal = zeros(3,num_wpts);
            for wpt=1:num_wpts
                target_index = current_target-1+wpt;
                goal_point(1:2) = opt_route(target_index,:);
%                 display(target_index)
                goal_point(3) = atan2(opt_route(target_index,2) - opt_route(target_index-1,2), opt_route(target_index,1) - opt_route(target_index-1,1));
                ddp_goal(:,wpt) = goal_point';
            end
            [pred_states, action_list_ddp, costs] = ddp_ddr(params, state, ddp_goal,init);
            fprintf('Got DDP solution\n');
            init.u_k = action_list_ddp;
            init_x_traj = pred_states;
            ddp_step = 1;
            calc_ddp = 1;
        end
        
        % Check if solution is in wall:
        for stp=1:round((length(pred_states)))
            if state.y < pred_states(2,stp)
                final_pos.y = ceil(pred_states(2,stp));
                final_pos_right.y = ceil(pred_states(2,stp) + sin(pred_states(3,stp))*1.5 - cos(pred_states(3,stp))*1);
                final_pos_left.y = ceil(pred_states(2,stp) + sin(pred_states(3,stp))*1.5 + cos(pred_states(3,stp))*1);
            else
                final_pos.y = ceil(pred_states(2,stp) );
                final_pos_right.y = ceil(pred_states(2,stp) + sin(pred_states(3,stp))*1.5 - cos(pred_states(3,stp))*1);
                final_pos_left.y = ceil(pred_states(2,stp) + sin(pred_states(3,stp))*1.5 + cos(pred_states(3,stp))*1);
            end
            if state.x < pred_states(1,stp)
                final_pos.x = ceil(pred_states(1,stp) );
                final_pos_right.x = ceil(pred_states(2,stp) + cos(pred_states(3,stp))*1.5 + sin(pred_states(3,stp))*1);
                final_pos_left.x = ceil(pred_states(2,stp) + sin(pred_states(3,stp))*1.5 - cos(pred_states(3,stp))*1);
            else
                final_pos.x = floor(pred_states(1,stp));
                final_pos_right.x = floor(pred_states(2,stp) + cos(pred_states(3,stp))*1.5 + sin(pred_states(3,stp))*1);
                final_pos_left.x = floor(pred_states(2,stp) + sin(pred_states(3,stp))*1.5 - cos(pred_states(3,stp))*1);
            end
%         end
%             if (observed_map(final_pos.y, final_pos.x) == 0)

%             if (observed_map(final_pos.y, final_pos.x) == 0 || observed_map(final_pos_right.y, final_pos_right.x) == 0 || observed_map(final_pos_left.y, final_pos_left.x) == 0)
            if (observed_map(final_pos.y, final_pos.x) == 0)

                % IN WALL!
                fprintf('Wall in the way!\n');
                if backup == 0
                   cnt = 1;
                   backup = 1;
                   current_target = current_target - 1;
                else
                    calc_ddp = 0;
                end
            end
        end
            if backup == 1
                cnt=cnt+1;
                fprintf('Backing up!\n');
               if cnt == 45
                    backup = 0;
                    recalculate = 1;
               end
            end
            if backup == 1
                action_list_ddp(1) = -2;
%                 cnt = 1;
%                 backup = 1;
            end
%             if backup == 1
%                 backup = 0;
%             end
        
        
        % check if a reasonable solution was found:
%         final_error.x = abs(pred_states(1,end) - opt_route(current_target,1));
%         final_error.y = abs(pred_states(2,end) - opt_route(current_target,2));
%         
%         if sqrt(final_error.x^2 + final_error.y^2) > 3.0
%             calc_ddp = 0;
%             action_list_ddp(1) = -2;
%         end

%         if at_goal
%             action_list_ddp(1) = 0;
%         end
        
        % current bridge state
        current_bridge_state = 0;
        for j = 1 : bridge_num
            bridge_row = map_struct.bridge_locations(2,j);
            bridge_col = map_struct.bridge_locations(1,j);
            bridge_value = observed_map(bridge_row, bridge_col);
            current_bridge_state = current_bridge_state + bridge_value;
        end
        
        %%% First action from DDP
        % action = action_list(k); 

        %%% My example policy: slight turn
        % Uncomment following line when we have the action list
%         action = (r_top - r_bot) * rand() + r_bot;
        
        %%% PID for angle direction
        % Get angle to goal
%         rad_error = -atan((state.y - map_struct.goal.y)/ (state.x - map_struct.goal.y));
%         deg_error = rad2deg(rad_error);
%         p = 1/180;
%         u = p * deg_error;
%         action = u;       
%         display(state)
%         display(ddp_goal)
%         display(state_error)
        action = action_list_ddp(1);
        ddp_step = ddp_step+1;
        
        
        % Notice how with this policy, when the car gets close to the
        % unknown bridge (in map_1), on the first map sample the bridge 
        % becomes solid black (closed), and on the second map sample the 
        % bridge becomes white (open). Until the bridge is either 1 (white)
        % or 0 (black), you should treat it as unknown. 
        %
        % For display purposes, the unknown bridge is shown as a gray shade
        % proportional to its probability of being closed.
        %
        %---------------------------------------
        
        % Execute the action and update observed_map
        [state, observed_map, flags] = motionModel(params, state, action, observed_map, map_struct.map_samples{i}, goal);

        % update bridge state
        updated_bridge_state = 0;
        for k = 1 : bridge_num
            updated_bridge_row = map_struct.bridge_locations(2,k);
            updated_bridge_col = map_struct.bridge_locations(1,k);
            updated_bridge_value = observed_map(updated_bridge_row, updated_bridge_col);
            updated_bridge_state = updated_bridge_state + updated_bridge_value;
        end
        
        if flags == 1
            Dataset.success = 1;
            name = strcat('map',string(Dataset.map), '_', string(Dataset.sample));
            save(name,'Dataset');           
            break;
        end
        
        if (DISPLAY_ON)
            display_environment;
        end

        % display some output
        
        % update differences parameter and update the index of action list
        diff = updated_bridge_state - current_bridge_state;
        k = k + 1;
        
        Dataset.time = [Dataset.time, state.moveCount];
        Dataset.path = [Dataset.path, [state.x, state.y, state.theta]'];
        Dataset.processing_time = toc;
        name = strcat('map',string(Dataset.map), '_', string(Dataset.sample));
        save(name,'Dataset');
        
        

        
        % pause if you'd like to pause at each step
        % pause;
        
    end
end
