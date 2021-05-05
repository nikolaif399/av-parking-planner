function plotstuff_nikolai(z,zd)

global L W 
% L and W are length and width of the car

w      = W/2;
l      = L/5;
x      = z(1);
y      = z(2);
theta  = z(3);
A      = sqrt(l^2 + w^2);
delta  = atan2(w,l);
x1     = [x-A*cos(delta+theta);y-A*sin(delta+theta)];
x2     = x1 + L*e(theta);
x3     = x2 + W*e(theta+pi/2);
x4     = x3 + L*e(theta+pi);
V      = [x1 x2 x3 x4 x1];
X      = V(1,:);
Y      = V(2,:);
line(X,Y,'linewidth',6,'color','red');
hold on
p1 = [x;y] + (w/2)*e(theta+pi/2); pw1 = p1 + (l)*e(theta); pw2 = p1 - (l/2)*e(theta);
p2 = [x;y] - (w/2)*e(theta+pi/2); pw3 = p2 + (l)*e(theta); pw4 = p2 - (l/2)*e(theta);

P1 = [pw1 pw2]; line(P1(1,:),P1(2,:),'linewidth',4,'color','black');
P2 = [pw3 pw4]; line(P2(1,:),P2(2,:),'linewidth',4,'color','black');

hold on


x      = zd(1);
y      = zd(2);
theta  = zd(3);
A      = sqrt(l^2 + w^2);
delta  = atan2(w,l);
x1     = [x-A*cos(delta+theta);y-A*sin(delta+theta)];
x2     = x1 + L*e(theta);
x3     = x2 + W*e(theta+pi/2);
x4     = x3 + L*e(theta+pi);
V      = [x1 x2 x3 x4 x1];
X      = V(1,:);
Y      = V(2,:);
line(X,Y,'linewidth',3,'color','black');

hold on
p1 = [x;y] + (w/2)*e(theta+pi/2); pw1 = p1 + (l)*e(theta); pw2 = p1 - (l/2)*e(theta);
p2 = [x;y] - (w/2)*e(theta+pi/2); pw3 = p2 + (l)*e(theta); pw4 = p2 - (l/2)*e(theta);

P1 = [pw1 pw2]; line(P1(1,:),P1(2,:),'linewidth',4,'color','black');
P2 = [pw3 pw4]; line(P2(1,:),P2(2,:),'linewidth',4,'color','black');


grid on
xlabel('X','interpreter','latex','fontsize',34);
ylabel('Y','interpreter','latex','fontsize',34);
axis equal
xlim([-3 15])
ylim([-3 15])
set(gca,'fontsize',24)
hold off
drawnow

end
function z = e(theta)
z  = [cos(theta);sin(theta)];
end