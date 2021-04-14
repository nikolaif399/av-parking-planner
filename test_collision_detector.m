% Sample car state
x = 10;
y = 10;
vehicle_width_ = 4;
vehicle_length_ = 8;
figure
for theta = -pi:0.01:pi

    % Compute car corner positions
    x1 = x + vehicle_width_/2*sin(theta);
    y1 = y - vehicle_width_/2*cos(theta);

    x2 = x - vehicle_width_/2*sin(theta);
    y2 = y + vehicle_width_/2*cos(theta);

    x3 = x2 + vehicle_length_*cos(theta);
    y3 = y2 + vehicle_length_*sin(theta);

    x4 = x1 + vehicle_length_*cos(theta);
    y4 = y1 + vehicle_length_*sin(theta);

    % Get line indices from mex 
    [indices1,~] = collision_debug(x1,y1,x2,y2);
    [indices2,~] = collision_debug(x2,y2,x3,y3);
    [indices3,~] = collision_debug(x3,y3,x4,y4);
    [indices4,~] = collision_debug(x4,y4,x1,y1);

    edge_indices = [indices1;indices2;indices3;indices4];

    grid = ones(20,20);

    % Fill in interior points
    xmin = min(edge_indices(:,1));
    xmax = max(edge_indices(:,1));
    indices = [];
    for col = xmin:xmax
        ymin = min(edge_indices(edge_indices(:,1) == col,2));
        ymax = max(edge_indices(edge_indices(:,1) == col,2));
        for row = ymin:ymax
            indices = [indices;col row];
        end
    end
    
  
    % Plot all points
    for i = 1:size(indices,1)
        grid(indices(i,1) + 1, indices(i,2) + 1) = 0;
    end


    % Upsample and draw ground truth lines
    n = 20;
    grid_big = imresize(grid, n,'nearest');
    
    hold off
    imshow(grid_big')
    hold on
    plot([x1*n,x2*n],[y1*n,y2*n],'r');
    plot([x2*n,x3*n],[y2*n,y3*n],'r');
    plot([x3*n,x4*n],[y3*n,y4*n],'r');
    plot([x4*n,x1*n],[y4*n,y1*n],'r');
    drawnow
    pause(0.01)
end