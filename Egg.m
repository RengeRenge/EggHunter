classdef Egg < handle
    properties
        center
        radius
        valid
    end

    methods
        function obj = Egg(center, radius, valid)
            if nargin > 0
                obj.center = center;
                obj.radius = radius;
                obj.valid = valid;
            else
                obj.center = [0, 0];
                obj.radius = 0;
                obj.valid = false;
            end
        end
    end
end