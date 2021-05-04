function [] = animateTrajectory(startState, goalStates, costmap,planStates,vehicleDims)
frameRate = 60;

record = 1;
if (record)
    v = VideoWriter('planner');
    v.FrameRate = frameRate;
    v.open()
end

fig = figure('units','normalized','outerposition',[0 0 1 1],'color','white'); 

plot(costmap, 'Inflation', 'off')
impixelinfo;
legend off

grid on
grid minor
set(gca,'xtick',[0:5*costmap.CellSize:costmap.MapSize(2)])
set(gca,'ytick',[0:5*costmap.CellSize:costmap.MapSize(1)])

length = vehicleDims(1);
width = vehicleDims(2);

g = hgtransform;
x = [0,0,length,length];
y = [-width/2,width/2,width/2,-width/2];
car = patch('XData',x,'YData',y,'FaceColor','blue','Parent',g);
set(car,'parent',g);


% Draw car at start state
startState = num2cell(startState);
[xs,ys,thetas] = startState{:};
trans=makehgtform('translate',[xs ys 0]);
rot=makehgtform('zrotate',thetas);
gStart = hgtransform;
set(gStart,'Matrix',trans*rot);
x = [0,0,length,length];
y = [-width/2,width/2,width/2,-width/2];
carStart = patch('XData',x,'YData',y,'FaceColor','none','EdgeColor','green','Parent',g);
carStart.FaceVertexAlphaData = 0.9;
set(carStart,'parent',gStart);

% Draw car at each goal state
for i = 1:size(goalStates,1)
    goalState = num2cell(goalStates(i,:));
    [xg,yg,thetag] = goalState{:};
    trans=makehgtform('translate',[xg yg 0]);
    rot=makehgtform('zrotate',thetag);
    gGoal = hgtransform;
    set(gGoal,'Matrix',trans*rot);
    x = [0,0,length,length];
    y = [-width/2,width/2,width/2,-width/2];
    carGoal = patch('XData',x,'YData',y,'FaceColor','none','EdgeColor','red','Parent',g);
    set(carGoal,'parent',gGoal);
end

for i = 1:size(planStates,1)
    pose = num2cell(planStates(i,:));
    
    [x,y,theta] = pose{:};
    
    trans=makehgtform('translate',[x y 0]);
    rot=makehgtform('zrotate',theta);
    set(g,'Matrix',trans*rot);
    
    if (record)
        writeVideo(v,getframe(gcf));
    end
    drawnow
    %plot(x,y, 'c-');
%     pause(1/frameRate);
end
pause(2)
if (record)
    v.close();
end

end

