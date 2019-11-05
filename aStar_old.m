function opt_route = aStar(input_map, state, goal)

% pre-porcess the wall by thickening the wall with one cell
copy_input_map(:,:) = input_map(:,:);
[m, n] = size(copy_input_map);

for y = 1 : m
    for x = 1 : n
        
        cell_state = input_map(y,x);
            if cell_state == 0
                if (y-1 > 1 || y-1 == 1) && (x-1 > 1 || x-1 == 1) && ...
                        (y+1 < 50 || y+1 == 50) && (x+1 < 50 || x+1 == 50)
                    copy_input_map(y-1,x-1) = 0;
                    copy_input_map(y-1,x) = 0;
                    copy_input_map(y-1,x+1) = 0;
                    copy_input_map(y+1,x+1) = 0;
                    copy_input_map(y+1,x-1) = 0;
                    copy_input_map(y+1,x) = 0;
                    copy_input_map(y,x+1) = 0;
                    copy_input_map(y,x-1) = 0;
                    
                elseif x-1 == 0 || y-1 == 0
                    copy_input_map(y+1,x+1) = 0;
                    copy_input_map(y+1,x) = 0;
                    copy_input_map(y,x+1) = 0;
                    
                elseif x+1 == 51 || y+1 == 51
                    copy_input_map(y-1,x-1) = 0;
                    copy_input_map(y-1,x) = 0;
                    copy_input_map(y,x-1) = 0;
                    
                end
            end
    end
end

updated_map(:,:) = copy_input_map(:,:);

OPEN = [];
CLOSE = [];
parent_children_list = [];
opt_route_list = [];

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
%     if parent_index == 0
%         break
%     end
    node_children = parent_children_list(parent_index,3:4);
    opt_route_list = [opt_route_list; node_children];
    
end

opt_route_list_flipped = flip(opt_route_list);

opt_route = opt_route_list_flipped;