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
        function obj = rangeImage(xy_image,skip,cleanFlag)
            if(nargin==3)
                n=0;
                for i=1:skip:length(xy_image)
                    n = n+1;
                    obj.rArray(n) = xy_image(i,1)*xy_image(i,1)+xy_image(i,2)*xy_image(i,2);
                    obj.tArray(n) = (i-1)*(pi/180);
                    obj.xArray(n) = xy_image(i,1);
                    obj.yArray(n) = xy_image(i,2);
                end
                obj.numPix = n;
                if cleanFlag; obj.removeBadPoints(); end;
            end
        end
        
        function removeBadPoints(obj)
            keep = find(obj.rArray<obj.maxUsefulRange & obj.rArray>obj.minUsefulRange);
            obj.rArray = obj.rArray(keep)
            obj.tArray = obj.tArray(keep)
            obj.xArray = obj.xArray(keep)
            obj.yArray = obj.yArray(keep)
            obj.numPix = length(obj.rArray)
            
        end
        
        function plotRvsTh(obj)
            obj.removeBadPoints();
            scatter(obj.tArray,obj.rArray);
            axis([0 2*pi 0 2]);
        end        
        
        function plotXvsY(obj)
            obj.removeBadPoints();
            figure(2);
            %axis([-maxRange maxRange -maxRange maxRange]);
            %axis manual;
            scatter(obj.yArray,obj.xArray);
            axis([-2 2 -2 2])
        end
        
        function [err num th] = findLineCandidate(obj,middle,maxLen)
            % STILL IN THE WORKS
            maxlen=0.13;
            m=middle;
            i=0;
            i1=m;
            i2=m;
            while(true)
                i1 = inc(obj,i1);
                i2 = dec(obj,i2);
                [x1 y1] = [obj.xArray(i1) obj.yArray(i1)];
                [x2 y2] = [obj.xArray(i2) obj.yArray(i2)];
                if(norm([(x2-x1) (y2-y1)])>maxlen)
                    break;
                end
                i=i+1;
            end
            num = 2*i;
            
            i1 
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
        
        
        
        
        
        