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
    methods
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
                [r2 , ~] = 
                lineMapLocalizer.closestPointOnLineSegment(pi,...
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
        
        function [errPlus0,J] = getJacobian(obj,poseIn,modelPts)
        % Computes the gradient of the error function
 
            errPlus0 = fitError(obj,poseIn,modelPts,false);
 
            eps = 0.001;
            % Calculation partial derivative for x
            dp = [eps ; 0.0 ; 0.0];
            newPose = pose(poseIn.getPose+dp);
            errPlus = fitError(obj,newPose,modelPts,false);
            J(:,1) = (errPlus-errPlus0)/eps;
            % Calculation partial derivative for y
            dp = [0.0 ; eps ; 0.0];
            newPose = pose(poseIn.getPose+dp);
            errPlus = fitError(obj,newPose,modelPts,false);
            J(:,2) = (errPlus-errPlus0)/eps;
            % Calculation partial derivative for th
            dp = [0.0 ; 0.0 ; eps];
            newPose = pose(poseIn.getPose+dp);
            errPlus = fitError(obj,newPose,modelPts,false);
            J(:,3) = (errPlus-errPlus0)/eps;
           
        end

    end