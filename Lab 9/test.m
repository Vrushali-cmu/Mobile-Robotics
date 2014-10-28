% Set up lines
p1 = [-2 ; -2];
p2 = [ 2 ; -2];
p3 = [ 2 ; 2];
p4 = [-2 ; 2];
lines_p1 = [p1 p2 p3 p4];
lines_p2 = [p2 p3 p4 p1];
%{
figure(1);
plot(lines_p1(1), lines_p1(2));
hold on
plot(lines_p1(1), lines_p1(2));
hold on
%}

nPts = 10;
x1 = -2.0*ones(1,nPts);
x2 = linspace(-2.0,2.0,nPts);
x3 = 2.0*ones(1,nPts);
y1 = linspace(0.0,2.0,nPts);
y2 = 2.0*ones(1,nPts);
y3 = linspace(2.0,0,nPts);

w = ones(1,3*nPts);
x1pts = [x1 x2 x3];
y1pts = [y1 y2 y3];
w1pts = w;
modelPts = [x1pts ; y1pts ; w1pts];
% pick a pose
dx = -0.1*rand();
dy = -0.1*rand();
dt = -0.05-0.2*rand();
thePose = pose(dx,dy,dt);


gain = 0.02;
errThresh = 0.002;
gradThresh = 0.0005;
maxIters = 200;

lml = lineMapLocalizer(lines_p1, lines_p2,gain,errThresh, gradThresh);
%[e,J] = lml.getJacobian(thePose,modelPts);
[success,outPose] = lml.refinePose(thePose,modelPts,maxIters);
