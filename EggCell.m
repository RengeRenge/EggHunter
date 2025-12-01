classdef EggCell < handle
    properties
        center
        valid
        egg
    end

    methods
        function obj = EggCell(center, valid, egg)
            if nargin > 0
                obj.center = center;
                obj.valid = valid;
                obj.egg = egg;
            else
                obj.center = [0, 0];
                obj.valid = false;
                obj.egg = Egg.empty;
            end
        end

        function result = hasEgg(obj)
            result = ~isempty(obj.egg);
        end
    end
end