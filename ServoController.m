classdef ServoController < handle
    properties (Access = private)
        servoObj
        pin
        arduinoConnections
        port
    end
    
    properties (SetAccess = private)
        currentAngle
        minPulse = 0.5e-3
        maxPulse = 2.5e-3
    end
    
    methods
        function obj = ServoController(pin, comPort)
            obj.pin = pin;
            
            persistent arduinoConnections;
            if isempty(arduinoConnections)
                arduinoConnections = containers.Map;
            end

            if ~isKey(arduinoConnections, comPort)
                try
                    arduinoObj = arduino(comPort, 'Mega2560');
                    arduinoConnections(comPort) = arduinoObj;
                    fprintf('Connected to Arduino (%s)\n', comPort);
                catch ME
                    error('Connected Arduino Error: %s', ME.message);
                end
            else
                fprintf('Connected to Arduino (%s)\n', comPort);
            end

            try
                arduinoObj = arduinoConnections(comPort);
                obj.port = comPort;
                obj.arduinoConnections = arduinoConnections;
                obj.servoObj = servo(arduinoObj, obj.pin, ...
                    'MaxPulseDuration', obj.maxPulse, ...
                    'MinPulseDuration', obj.minPulse);
            catch ME
                error('Create Servo Error: %s', ME.message);
            end
        end
        
        function moveTo(obj, angle)
            angle = (pi-angle)/pi;
            if angle < 0
                angle = 0;
            elseif angle > 1
                angle = 1;
            end
            writePosition(obj.servoObj, angle);
            % actualPosition = readPosition(obj.servoObj);
            obj.currentAngle = angle;
        end

        function delete(obj)
            obj.servoObj.disconnect();
            clear obj.arduinoConnections(obj.port);
        end
    end
end