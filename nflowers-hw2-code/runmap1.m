startQ = [pi/2 pi/4 pi/2 pi/4 pi/2];
goalQ = [pi/8 3*pi/4 pi 0.9*pi 1.5*pi];

% Map 2 valid config
startQ(2) = startQ(2) + 0.5;
startQ(3) = startQ(3) + 0.6;

plannerId = 1;
tic
runtest('map2.txt', startQ, goalQ, plannerId);
toc