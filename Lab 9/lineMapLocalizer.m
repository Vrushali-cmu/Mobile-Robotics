classdef lineMapLocalizer < handle
 %mapLocalizer A class to match a range scan against a map in 
 % order to find the true location of the range scan relative to 
 % the map.
 
 properties(Constant)
    maxErr = 0.05; % 5 cm
    minPts = 5; % min # of points that must match
 end
 
 properties(Access = private)
 end
 
 properties(Access = public)
    lines_p1 = [];
    lines_p2 = [];
    gain = 0.0;
    errThresh = 0.0;
    gradThresh = 0.0;
 end

 methods(Static = true)
     function [rad2 , po] = closestPointOnLineSegment(pi,p1,p2)
 % Find point po on a line segment p1-p2 closest to a given 
 % point pi and return the closest point and the square of
% the distance to it. The line segment has endpoints p1 
% and p2 (column vectors) and the point is pi. If the 
% closest point is an endpoint, returns infinity for rad2
% because such points are bad for lidar matching
% localization.
 dx13 = pi(1) - p1(1);
 dy13 = pi(2) - p1(2);
 dx12 = p2(1) - p1(1);
 dy12 = p2(2) - p1(2); 
 dx23 = pi(1) - p2(1);
 dy23 = pi(2) - p2(2);
 v1 = [dx13 ; dy13];
 v2 = [dx12 ; dy12];
 v3 = [dx23 ; dy23];
 v1dotv2 = dot(v1,v2);
 v2dotv2 = dot(v2,v2);
 v3dotv2 = dot(v3,v2);
 if v1dotv2 > 0.0 && v3dotv2 < 0.0 % Closest is on segment
 scale = v1dotv2/v2dotv2;
po = v2*scale + [p1(1) ; p1(2)];
 dx = pi(1)-po(1);
 dy = pi(2)-po(2);
rad2 = dx*dx+dy*dy;
 elseif v1dotv2 <= 0.0 % Closest is first endpoint
 po = [p1(1) ; p1(2)];
rad2 = inf;
 else % Closest is second endpoint
 po = [p2(1) ; p2(2)];
 rad2 = inf;
 end
     end
 
 end
     
 methods(Static = false)
 function obj = lineMapLocalizer(lines_p1,lines_p2,gain,errThresh,gradThresh)
    % create a lineMapLocalizer
    obj.lines_p1 = lines_p1;
    obj.lines_p2 = lines_p2;
    obj.gain = gain;
    obj.errThresh = errThresh;
    obj.gradThresh = gradThresh;
 end

 function ro2 = closestSquaredDistanceToLines(obj,pi)
    % Find the squared shortest distance from pi to any line 
    % segment in the supplied list of line segments. p1 is the 
    % array of start point and p2 is the array of end points.
    ro2 = inf;
    for i = 1:size(obj.lines_p1,2)
        [r2 , ~] = lineMapLocalizer.closestPointOnLineSegment(pi,...
                    obj.lines_p1(:,i),obj.lines_p2(:,i));
        if(r2 < ro2); ro2 = r2; end; 
    end
 end
 
 function ids = throwOutliers(obj,pose,ptsInModelFrame)
    % Find ids of outliers in a scan. 
    ids = [];
    worldPts = pose.bToA()*ptsInModelFrame;
    for i = 1:size(worldPts,2)
        r2 = obj.closestSquaredDistanceToLines(worldPts(:,i));
        if(sqrt(r2) < obj.maxError)
            ids = [ids i];
        end 
    end
 end
 
 function avgErr = fitError(obj,pose,ptsInModelFrame,printErr)
    % Find the standard deviation of perpendicular distances of
    % all points to all lines
    %transform the points
    worldPts = pose.bToA()*ptsInModelFrame;
 
    err = 0.0;
    num = 0;
    for i = 1:size(worldPts,2)
        r2 = obj.closestSquaredDistanceToLines(worldPts(:,i));
        if(r2 == Inf)
            continue;
        end
        err = err + r2;
        num = num + 1;
        if(printErr)
            fprintf('i: %d x:%f y:%f val:%f\n',i,...
            worldPts(1,i),worldPts(2,i),r2);
        end
    end
    if(num > lineMapLocalizer.minPts)
        avgErr = sqrt(err)/num;
    else
        avgErr = inf;
    end
 end
 
 function [success, outPose] =  refinePose(obj,inPose,ptsInModelFrame,maxIters)
    % refine robot pose in world (inPose) based on lidar
    % registration. Terminates if maxIters iterations is
    % exceeded or if insufficient points match the lines.
    % Even if the minimum is not found, outPose will contain 
    % any changes that reduced the fit error. Pose changes that
    % increase fit error are not included and termination 
    % occurs thereafter.
    error = inf;
    newPose = pose(inPose.x,inPose.y,inPose.th);
    n = 0;
    err_array = zeros(1,500);
    figure(2);
    prevError = [];
    while error>obj.errThresh
        n = n+1;
        [error,J] = obj.getJacobian(newPose,ptsInModelFrame);
        dx = -obj.gain*J(1);
        dy = -obj.gain*J(2);
        dt = -obj.gain*J(3);
        newPose = pose(newPose.x+dx, newPose.y+dy, newPose.th+dt);
        worldPts = newPose.bToA()*ptsInModelFrame;
        
        clf;
        plot(obj.lines_p1(1,:),obj.lines_p1(2,:));
        hold on
        plot(obj.lines_p2(1,:),obj.lines_p2(2,:));
        hold on
        scatter(worldPts(1,:),worldPts(2,:),'x');
        hold on
        
        err_array(n) = error;
        if n>maxIters
            success = 0;
            outPose = pose(newPose.x,newPose.y,newPose.th);
            return;
        end
        if isempty(prevError)
            prevError = inf;
            continue;
        end
        dE = abs(error-prevError);
        if dE<obj.gradThresh
            break;
        end
        
        pause(0.1);
        
        
    end
    success = 1;
    outPose = pose(newPose.x,newPose.y,newPose.th);
    %figure(3);
    %plot(err_array);
    
 end

 
 function [errPlus0,J] = getJacobian(obj,poseIn,modelPts)
    % Computes the gradient of the error function
 
    errPlus0 = fitError(obj,poseIn,modelPts,false);
 
    eps = 0.001;
    
    J = zeros(1,3);
    dp = [eps ; 0.0 ; 0.0];
    newPose = pose(poseIn.getPose+dp);
    dEx = fitError(obj,newPose,modelPts,false)-errPlus0;
    J(:,1) = dEx/eps;
    
    dp = [0.0 ; eps ; 0.0];
    newPose = pose(poseIn.getPose+dp);
    dEy = fitError(obj,newPose,modelPts,false)-errPlus0;
    J(:,2) = dEy/eps;
    
    dp = [0.0 ; 0.0 ; eps];
    newPose = pose(poseIn.getPose+dp);
    dEth = fitError(obj,newPose,modelPts,false)-errPlus0;
    J(:,3) = dEth/eps;
 end
 
 end
end