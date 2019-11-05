function opt_route = aStar_fixed(input_map, state, goal, map_struct)

updated_map(:,:) = input_map(:,:);

% check the pass way of the gate and sure the optimal route will pass
% through the center of the gate
bridge_array = map_struct.bridge_locations;
[~, n] = size(bridge_array);

for index_1 = 1 : n
    b_x = bridge_array(1, index_1);
    b_y = bridge_array(2, index_1);
    
    btm = updated_map(b_y+2,b_x);
    rgt = updated_map(b_y,b_x+2);
    
    if btm == 0 
        updated_map(b_y+1,b_x) = 0;
        updated_map(b_y-1,b_x) = 0;
    end
    
    if rgt == 0 
        updated_map(b_y,b_x+1) = 0;
        updated_map(b_y,b_x-1) = 0;
    end
    
end

OPEN = [];
CLOSE = [];
parent_children_list = [];
opt_route_list = [];
final_list = [];

node_start = [round(state.x, 0), round(state.y, 0)];
node_goal = [goal.x, goal.y];

start_h_cost = round(10 * sqrt(sum((node_start - node_goal).^2)), 0);
first_node_in = [round(state.x, 0), round(state.y, 0), start_h_cost];

OPEN = [OPEN; first_node_in];

% while open list is not empty
while isempty(OPEN) == 0 
    
    % choose the node with lowest f value
    [~, c_index] = min(OPEN(:,3));
    node_current = OPEN(c_index, 1:2);
    
    % remove current node from open list and move it to close list
    OPEN(c_index, :) = [];
    CLOSE = [CLOSE; node_current];
    
    % leave the loop if we reach the goal node
    if node_current(1,1) == node_goal(1,1) && node_current(1,2) == node_goal(1,2)
        break
    end
    
    % generate the neighbor nodes of the current node
    c_x = node_current(1);
    c_y = node_current(2);
    node_neighbor_list = [c_x-1 c_y-1; c_x c_y-1; c_x+1 c_y-1; c_x-1 c_y; c_x+1 c_y; c_x-1 c_y+1; c_x c_y+1; c_x+1 c_y+1];
    
    % g-cost of current node
    g_cost_current = round(10 * sqrt(sum((node_current - node_start).^2)), 0);
    
    % check each neighbor
    for i = 1 : 8
        
        % set the neighbor node
        node_neighbor = node_neighbor_list(i,:);
        
        % g-cost of ith neighbor node
        g_cost_i = round(10 * sqrt(sum((node_neighbor - node_start).^2)), 0);
        
        % h-cost of ith neighbor node
        h_cost_i = round(10 * sqrt(sum((node_neighbor - node_goal).^2)), 0);
        
        % movement cost from current node to ith neighbor node
        movement_cost = round(10 * sqrt(sum((node_neighbor - node_current).^2)), 0);
        
        cost = g_cost_current + movement_cost;
        
        % check if the i-th neighbor node is in the open list or not
        if isempty(OPEN) == 1
            in_open = 0;
        else
            [~, in_open] = ismember(node_neighbor, OPEN(:,1:2), 'row');
        end
        
        % check if the i-th neighbor node is in the close list or not 
        if isempty(CLOSE) == 1
            in_close = 0;
        else
            [~, in_close] = ismember(node_neighbor, CLOSE(:,1:2), 'row');
        end
        
        % remove neighbor node from open lsit because new path is better
        if in_open ~= 0 && cost < g_cost_i
            OPEN(in_open, :) = [];
        end
        
        % remove neighbor node from close list. this should never happen
        if in_close ~= 0 && cost < g_cost_i
            CLOSE(in_close, :) = [];
        end
        
        % neighbor node not in open list or close list
        if (in_open == 0) && (in_close == 0)
            
            x_n = node_neighbor(1);
            y_n = node_neighbor(2);
            map_status = updated_map(y_n, x_n);
            
            % if the cell contains wall or unopen gate [0,1)
            if map_status == 0
                continue
            else
                % set g-cost of ith neighbor to cost
                g_cost_i = cost;
                % f-cost for the ith neighbor node
                f_cost_i = g_cost_i + h_cost_i;
                node_input = [node_neighbor(1), node_neighbor(2), f_cost_i];
                % add neighbor node to open list
                OPEN = [OPEN; node_input];
                % set neighbor node's parent to current node
                parent_children_pair = [node_neighbor, node_current];
                parent_children_list = [parent_children_list; parent_children_pair];
            end
            
        end
        
    end
end

node_children = [goal.x, goal.y];
opt_route_list = [opt_route_list; node_children];

while node_children(1,1) ~= node_start(1,1) || node_children(1,2) ~= node_start(1,2)
    
    [~, parent_index] = ismember(node_children, parent_children_list(:,1:2), 'row');
    node_children = parent_children_list(parent_index,3:4);
    opt_route_list = [opt_route_list; node_children];
    
end

opt_route_list_flipped = flip(opt_route_list);
[num_point, ~] = size(opt_route_list_flipped);

final_list = [final_list; opt_route_list_flipped(1,:)];

% shift the route away from the wall
% location of the positions:
%
%       1   top 2     3
%   left 4  cell   right 5
%       6  bottom 7   8
%

for index_2 = 2 : num_point
    
    % x, y of current point in the route list
    x_p = opt_route_list_flipped(index_2,1); 
    y_p = opt_route_list_flipped(index_2,2);
    
    % x, y of previous point in the route list
    x_p_0 = opt_route_list_flipped(index_2-1,1);
    y_p_0 = opt_route_list_flipped(index_2-1,2);
    
    % change of the cell coordinates for direction
    del_x = x_p - x_p_0;
    del_y = y_p - y_p_0;
    
    % for each point check 
    top_index = 0;
    btm_index = 0;
    lft_index = 0;
    rgt_index = 0;
    tl_index = 0;
    tr_index = 0;
    bl_index = 0;
    br_index = 0;
    
    top_cell = input_map(y_p-1, x_p);
    lft_cell = input_map(y_p, x_p-1);
    rgt_cell = input_map(y_p, x_p+1);
    btm_cell = input_map(y_p+1, x_p);
    
    x_1 = x_p - 1;
    x_2 = x_p;
    x_3 = x_p + 1;
    x_4 = x_1;
    x_5 = x_3;
    x_6 = x_1;
    x_7 = x_2;
    x_8 = x_3;
    
    y_1 = y_p - 1;
    y_2 = y_1;
    y_3 = y_1;
    y_4 = y_p;
    y_5 = y_4;
    y_6 = y_p + 1;
    y_7 = y_6;
    y_8 = y_6;
    
    tl_cell = input_map(y_1, x_1);
    tr_cell = input_map(y_3, x_3);
    bl_cell = input_map(y_6, x_6);
    br_cell = input_map(y_8, x_8);
    
    cell_1 = [x_1, y_1];
    cell_2 = [x_2, y_2];
    cell_3 = [x_3, y_3];
    cell_4 = [x_4, y_4];
    cell_5 = [x_5, y_5];
    cell_6 = [x_6, y_6];
    cell_7 = [x_7, y_7];
    cell_8 = [x_8, y_8];
    input_point = [x_p, y_p];
    
    % check if cell is in the final_list
    [~, cell_1_idx] = ismember(cell_1, final_list, 'row');
    [~, cell_2_idx] = ismember(cell_2, final_list, 'row');
    [~, cell_3_idx] = ismember(cell_3, final_list, 'row');
    [~, cell_4_idx] = ismember(cell_4, final_list, 'row');
    [~, cell_5_idx] = ismember(cell_5, final_list, 'row');
    [~, cell_6_idx] = ismember(cell_6, final_list, 'row');
    [~, cell_7_idx] = ismember(cell_7, final_list, 'row');
    [~, cell_8_idx] = ismember(cell_8, final_list, 'row');
    
    % check if cell is in the opt_route_list_flipped
    [~, cell_1_check] = ismember(cell_1, opt_route_list_flipped, 'row');
    [~, cell_3_check] = ismember(cell_3, opt_route_list_flipped, 'row');
    [~, cell_6_check] = ismember(cell_6, opt_route_list_flipped, 'row');
    [~, cell_8_check] = ismember(cell_8, opt_route_list_flipped, 'row');
    
    [~, input_idx] = ismember(input_point, final_list, 'row');
    
    if top_cell == 0
        top_index = 1;
    end
    if btm_cell == 0
        btm_index = 1;
    end
    if lft_cell == 0
        lft_index = 1;
    end
    if rgt_cell == 0
        rgt_index = 1;
    end
    if tl_cell == 0
        tl_index = 1;
    end
    if tr_cell == 0
        tr_index = 1;
    end
    if bl_cell == 0
        bl_index = 1;
    end
    if br_cell == 0
        br_index = 1;
    end
    
    sum_index = top_index + btm_index + lft_index + rgt_index;
    sum_index_corner = tl_index + tr_index + bl_index + br_index;
    sum_index_total = sum_index + sum_index_corner;
    
    if sum_index_total == 0
        if input_idx == 0
            final_list = [final_list; input_point];
        end
    end
    
    if top_index == 1 && sum_index == 1
        if del_x > 0
            if cell_6_idx == 0 
                final_list = [final_list; cell_6];
            end
            if cell_7_idx == 0
                final_list = [final_list; cell_7];
            end
            if cell_8_idx == 0
                final_list = [final_list; cell_8];
            end
        else
            if cell_8_idx == 0
                final_list = [final_list; cell_8];
            end
            if cell_7_idx == 0
                final_list = [final_list; cell_7];
            end
            if cell_6_idx == 0
                final_list = [final_list; cell_6];
            end
        end
    end
    
    if btm_index == 1 && sum_index == 1
        if del_x > 0
            if cell_1_idx == 0
                final_list = [final_list; cell_1];
            end
            if cell_2_idx == 0
                final_list = [final_list; cell_2];
            end
            if cell_3_idx == 0
                final_list = [final_list; cell_3];
            end
        else
            if cell_3_idx == 0 
                final_list = [final_list; cell_3];
            end
            if cell_2_idx == 0
                final_list = [final_list; cell_2];
            end
            if cell_1_idx == 0
                final_list = [final_list; cell_1];
            end
        end
    end
    
    if lft_index == 1 && sum_index == 1
        if del_y > 0
            if cell_3_idx == 0 
                final_list = [final_list; cell_3];
            end
            if cell_5_idx == 0
                final_list = [final_list; cell_5];
            end
            if cell_8_idx == 0 
                final_list = [final_list; cell_8];
            end
        else
            if cell_8_idx == 0
                final_list = [final_list; cell_8];
            end
            if cell_5_idx == 0
                final_list = [final_list; cell_5];
            end
            if cell_3_idx == 0 
                final_list = [final_list; cell_3];
            end
        end
    end
    
    if rgt_index == 1 && sum_index == 1
        if del_y > 0
            if cell_1_idx == 0 
                final_list = [final_list; cell_1];
            end
            if cell_4_idx == 0
                final_list = [final_list; cell_4];
            end
            if cell_6_idx == 0
                final_list = [final_list; cell_6];
            end
        else
            if cell_6_idx == 0 
                final_list = [final_list; cell_6];
            end
            if cell_4_idx == 0
                final_list = [final_list; cell_4];
            end
            if cell_1_idx == 0 
                final_list = [final_list; cell_1];
            end
        end
    end
    
     % check if cell is in the final_list, AGAIN!
    [~, cell_1_idx] = ismember(cell_1, final_list, 'row');
    [~, cell_3_idx] = ismember(cell_3, final_list, 'row');
    [~, cell_6_idx] = ismember(cell_6, final_list, 'row');
    [~, cell_8_idx] = ismember(cell_8, final_list, 'row');
%     [~, cell_7_idx] = ismember(cell_7, final_list, 'row');
%     [~, cell_4_idx] = ismember(cell_4, final_list, 'row');
%     [~, cell_5_idx] = ismember(cell_5, final_list, 'row');
%     [~, cell_2_idx] = ismember(cell_2, final_list, 'row');
    
    % for top-right corner
    if tr_index == 1 && sum_index_corner == 1 
        if cell_6_idx == 0 && cell_6_check == 0
            final_list = [final_list; cell_6];
        end 
    end
    
    % for top-left corner
    if tl_index == 1 && sum_index_corner == 1 
        if cell_8_idx == 0 && cell_8_check == 0
            final_list = [final_list; cell_8];
        end
    end
    
    % for bottom-right corner
    if br_index == 1 && sum_index_corner == 1
        if cell_1_idx == 0 && cell_1_check == 0
            final_list = [final_list; cell_1];
        end
    end
    
    % for bottom-left corner
    if bl_index == 1 && sum_index_corner == 1
        if cell_3_idx == 0 && cell_3_check == 0
            final_list = [final_list; cell_3];
        end
    end
    
end

opt_route = final_list;