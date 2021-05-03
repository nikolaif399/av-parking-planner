
plannerId = 2;

LINKLENGTH_CELLS=10;
envmap = load('map2.txt');

numGoodConfigs = 0;
N = 100;

qlo = [0 pi/8 pi/4 pi/8 pi/4];
qhi = [pi 2*pi 1.5*pi 1.5*pi 2*pi];

startQ = [pi/2 pi/4 pi/2 pi/4 pi/2];
goalQ = [pi/8 3*pi/4 pi 0.9*pi 1.5*pi];

plannerTimesMs = zeros(1,N);
pathLengths = zeros(1,N);
numVertices = zeros(1,N);
while(numGoodConfigs < N)
    startQ = qlo + (qhi - qlo) .* rand([1 5]);
    goalQ = qlo + (qhi - qlo) .* rand([1 5]);
    
    tic
    [armplan, armplanlength, numVertex] = planner(envmap, startQ, goalQ, plannerId);
    plannerTimesMs(numGoodConfigs+1) = toc*1000;
    pathLengths(numGoodConfigs+1) = armplanlength;
    numVertices(numGoodConfigs+1) = numVertex;

    if armplanlength > 1
        %disp(armplanlength);
        %close all;
        %animatePlan(envmap,armplan,LINKLENGTH_CELLS);
        numGoodConfigs = numGoodConfigs + 1
    end
end

%disp(plannerTimesMs)
%disp(pathLengths)
%disp(numVertices)

fprintf("Planning times: %f, %f, %f, %f\n",min(plannerTimesMs),max(plannerTimesMs),mean(plannerTimesMs),std(plannerTimesMs)); 
fprintf("Path lengths: %d, %d, %f, %f\n",min(pathLengths),max(pathLengths),mean(pathLengths),std(pathLengths)); 
fprintf("Num vertices: %d, %d, %f, %f\n",min(numVertices),max(numVertices),mean(numVertices),std(numVertices)); 


maxTime = max(plannerTimesMs);

numUnder = 0;

ts = 0:0.05:maxTime;
ys = [];
for t = ts
  ys = [ys length(plannerTimesMs(plannerTimesMs < t))];
end
ys = ys/N;

figure
patch([ts fliplr(ts)], [ys zeros(size(ys))], [0.6 0.8 1])
ylim([0 1])
xlim([0,maxTime])
xlabel("Planning time(ms)")
ylabel("Ratio of plans completed")
title("Area under the curve with plannerId " + num2str(plannerId))




