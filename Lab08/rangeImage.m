classdef rangeImage < handle
    properties(Constant)
        maxUsefulRange = 2.0;
        minUsefulRange = 0.05;
        maxRangeForTarget = 1.0;
    end
    
    properties (Access = public)
        rArray = [];
        tArray = [];
        xArray = [];
        yArray = [];
        numPix;
    end
    
    methods(Access = public)
        function obj = rangeImage(ranges,skip,cleanFlag)
            if(nargin==3)
                n=0;
                for i=1:skip:length(ranges)
                    n = n+1;
                    obj.rArray(n) = ranges(i);
                    obj.tArray(n) = (i-1)*(pi/180);
                    obj.xArray(n) = ranges(i)*cos(obj.tArray(n));
                    obj.yArray(n) = ranges(i)*sin(obj.tArray(n));
                end
                obj.numPix = n;
                if cleanFlag; obj.removeBadPoints(); end;
            end
        end
        
        function removeBadPoints(obj)
            del1 = find(obj.rArray>obj.maxUsefulRange);
            del2 = find(obj.rArray<obj.minUsefulRange);
            obj.rArray(del1) = [];
            obj.rArray(del2) = [];
            obj.tArray(del1) = [];
            obj.tArray(del2) = [];
            obj.xArray(del1) = [];
            obj.xArray(del2) = [];
            obj.yArray(del1) = [];
            obj.yArray(del2) = [];
        end
        
        function plotRvsTh(obj, maxRange)
            obj.removeBadPoints();
            figure(1);
            axis([-180 180 -maxRange maxRange]);
            %axis manual;
            scatter(obj.tArray,obj.rArray,'.');
        end
        
        function plotXvsY(obj, maxRange)
            obj.removeBadPoints();
            figure(1);
            axis([-maxRange maxRange -maxRange maxRange]);
            axis manual;
            scatter(obj.yArray,obj.xArray,'.');
        end
        
        function [err num th] = findLineCandidate(obj,middle,maxLen)
            % NEED TO FILL THIS IN.
        end
        
        function num = numPixels(obj)
            num = obj.numPix;
        end
        
        function out = inc(obj,in)
            out = indexAdd(obj,in,1);
        end
        
        function out = dec(obj,in)
            out = indexAdd(obj,in,-1);
        end
        
        function out = indexAdd(obj,a,b)
            out = mod((a-1)+b, obj.numpix)+1;
        end
    end
end
        
        
        
        
        
        