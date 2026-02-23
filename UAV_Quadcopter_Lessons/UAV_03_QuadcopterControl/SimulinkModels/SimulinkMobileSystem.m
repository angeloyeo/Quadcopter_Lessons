classdef SimulinkMobileSystem < matlab.System
    % SimulinkMobileSystem
    %

    % Public, tunable properties
    properties (Nontunable)
        AccelerationEnabled (1,1) logical = true % Acceleration
        OrientationEnabled (1,1) logical = false  % Orientation
        PositionEnabled (1,1) logical = false % Position

        CameraEnabled (1,1) logical = false % Camera
        CameraSelected (1,1) string {mustBeMember(CameraSelected,{'back','front'})} = "back";
        Frequency (1,1) double {mustBeBetween(Frequency,1,100)} = 10 % Frequency
    end

    % Pre-computed constants or internal states
    properties (Access = private)
        mdev
        cam
    end

    methods (Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.mdev = mobiledev;

            obj.mdev.AccelerationSensorEnabled = obj.AccelerationEnabled;
            obj.mdev.OrientationSensorEnabled = obj.OrientationEnabled;
            obj.mdev.PositionSensorEnabled = obj.PositionEnabled;

            if obj.CameraEnabled
                obj.cam = obj.mdev.camera(obj.CameraSelected);
            end
            obj.mdev.SampleRate = obj.Frequency;
            obj.mdev.Logging = true;
        end

        function [Acceleration, Orientation, Position, Image] = stepImpl(obj)
            % Default placeholders (unused outputs can be left empty)
            Acceleration = [];
            Orientation  = [];
            Position     = [];
            Image = [];

            % Collect enabled outputs in compile-time ordering
            outs = {};
            if obj.AccelerationEnabled
                acc = obj.mdev.Acceleration;
                if isempty(acc), acc = [0 0 0]; end
                outs{end+1} = acc;
            end
            if obj.OrientationEnabled
                ori = obj.mdev.Orientation;
                if isempty(ori), ori = [0 0 0]; end
                outs{end+1} = ori;
            end

            if obj.PositionEnabled
                pos = [obj.mdev.Latitude, obj.mdev.Longitude];
                if isempty(pos), pos = [0 0]; end
                outs{end+1} = pos;
            end

            if obj.CameraEnabled
                outs{end+1} = obj.cam.snapshot("immediate");
            end

            % Assign collected outputs into the three return variables.
            % Simulink will use only the first getNumOutputsImpl(obj) of them.
            if numel(outs) >= 1, Acceleration = outs{1}; end
            if numel(outs) >= 2, Orientation  = outs{2}; end
            if numel(outs) >= 3, Position     = outs{3}; end
            if numel(outs) >= 4, Image        = outs{4}; end
        end


        function resetImpl(obj)
            % Initialize / reset internal properties
        end

    function names = getOutputNamesImpl(obj)
            names = string.empty(1,0);
            if obj.AccelerationEnabled
                names(end+1) = "Acceleration";
            end
            if obj.OrientationEnabled
                names(end+1) = "Orientation";
            end
            if obj.PositionEnabled
                names(end+1) = "Position";
            end
                        if obj.CameraEnabled
                names(end+1) = "Image";
            end
    end

        function [out1, out2, out3,out4] = getOutputSizeImpl(obj)
            % Build a cell/array for the enabled outputs in the same order
            sizes = {};
            if obj.AccelerationEnabled, sizes{end+1} = [1 3]; end
            if obj.OrientationEnabled,  sizes{end+1} = [1 3]; end
            if obj.PositionEnabled,     sizes{end+1} = [1 2]; end
            if obj.CameraEnabled,       sizes{end+1} = [640 480 3]; end

            % Return matching number of outputs. If fewer than three outputs,
            % only first N returned values are used by Simulink.
            % Fill remaining with [] to satisfy signature.
            while numel(sizes) < 4, sizes{end+1} = []; end
            out1 = sizes{1}; out2 = sizes{2}; out3 = sizes{3}; out4 = sizes{4};
        end

        function [out1, out2, out3,out4] = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            out1 = "double";
            out2 = "double";
            out3 = "double";
            out4 = "uint8";

            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end

        function [out1, out2, out3, out4] = isOutputComplexImpl(obj)
            % Return true for each output port with complex data
            out1 = false;
            out2 = false;
            out3 = false;
            out4 = false;

            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end

        function [out1, out2, out3,out4] = isOutputFixedSizeImpl(obj)
            % Return true for each output port with fixed size
            out1 = true;
            out2 = true;
            out3 = true;
            out4 = true;

            % Example: inherit fixed-size status from first input port
            % out = propagatedInputFixedSize(obj,1);
        end

        function num = getNumOutputsImpl(obj)
            % Order: 1=Acceleration, 2=Orientation, 3=Position
            num = double(obj.AccelerationEnabled) + ...
                double(obj.OrientationEnabled) + ...
                double(obj.PositionEnabled) + ...
                double(obj.CameraEnabled);
        end

        function icon = getIconImpl(~)
            % Define icon for System block
            icon = matlab.system.display.Icon('SimulinkMobile.jpg');
        end

        function sts = getSampleTimeImpl(obj)
            sts = createSampleTime(obj,'Type','Discrete','SampleTime',1/obj.Frequency);
        end
    end

    methods (Static, Access = protected)
        function simMode = getSimulateUsingImpl
            simMode = "Interpreted execution";
        end 
    end
end
