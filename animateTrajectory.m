function [] = animateTrajectory(costmap,planStates,vehicleDims)
frameRate = 30;

v = VideoWriter('planner');
v.FrameRate = frameRate;
v.open()

figure(1)
plot(costmap, 'Inflation', 'off')
legend off

length = vehicleDims(1);
width = vehicleDims(2);

g = hgtransform;
x = [0,0,length,length];
y = [-width/2,width/2,width/2,-width/2];
car = patch('XData',x,'YData',y,'FaceColor','red','Parent',g);
set(car,'parent',g);

for i = 1:size(planStates,1)
    pose = num2cell(planStates(i,:));
    [x,y,theta,steer] = pose{:};
    
    trans=makehgtform('translate',[x y 0]);
    rot=makehgtform('zrotate',theta);
    set(g,'Matrix',trans*rot);
    
    writeVideo(v,getframe(gcf));
    
    %plot(x,y, 'c-');
    pause(1/frameRate);
end
v.close();

end

