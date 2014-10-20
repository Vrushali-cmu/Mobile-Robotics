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
            %obj.removeBadPoints();
            scatter(obj.tArray,obj.rArray);
            axis([0 2*pi 0 2]);
        end        
        
        function plotXvsY(obj)
            %obj.removeBadPoints();
            %axis([-maxRange maxRange -maxRange maxRange]);
            %axis manual;
            scatter(obj.yArray,obj.xArray);
            axis([-2 2 -2 2])
        end
        
        function [l_err num pose] = findLineCandidate(obj,middle)
            middle_index = find(obj.tArray==middle);
            
            %initializing values of dist,i1 and i2 and i1_final and
            %                                               i2_final
            i1 = obj.inc(middle_index);
            i2 = obj.dec(middle_index);
            num = 2;
            i1_final = i1;
            i2_final = i2;
            dist=0;
            while(true)
                x1 = obj.xArray(i1);
                y1 = obj.yArray(i1);
                x2 = obj.xArray(i2);
                y2 = obj.yArray(i2);
                dist = norm([x1-x1 y1-y2])
                if(dist<0.2 & dist>0)
                    i1_final = i1;
                    i2_final = i2;
                    i1 = obj.inc(i1);
                    i2 = obj.dec(i2);
                    num = num+2;
                else
                    break;
                end
            end
            %now we have 2 end points. Time to plot a line that passes
            %through them
            
            
            x = [obj.xArray(i1_final),obj.xArray(i2_final)];
            y = [obj.yArray(i1_final),obj.yArray(i2_final)];
            %plot(y,x);
            distance = norm([x(1)-x(2) y(1)-y(2)]);
            l_err = abs(0.125-distance);
            th = ((y(1)-y(2))/(x(1)-x(2)));
            pose = [obj.xArray(middle_index) obj.yArray(middle_index) th];
            
            
            
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
            out = mod((a-1)+b, obj.numPix)+1;
        end
    end
end

        
        
        
        
        
        