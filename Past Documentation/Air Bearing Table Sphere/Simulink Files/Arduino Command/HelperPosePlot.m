classdef HelperPosePlot< matlab.System
    %Visualize orientation

    %   Copyright 2022 The MathWorks, Inc.

    %#codegen
    properties (Nontunable)
        % Orientation Format
        OrientationFormat = 'Quaternion';
    end

    properties (Constant,Hidden)
        OrientationFormatSet = matlab.system.StringSet({'Quaternion', 'Rotation matrix'});
    end

    properties (Access = protected)
        visObj
    end

    methods(Access = protected)
        function setupImpl(obj)
            if coder.target('MATLAB')
                obj.visObj = poseplot;
                xlabel("North-x (m)")
                ylabel("East-y (m)")
                zlabel("Down-z (m)");
                title("Orientation using AHRS filter and IMU sensor");
            end
        end

        function stepImpl(obj,orient)
            if coder.target('MATLAB')
                if isvalid(obj.visObj)
                    if strcmpi(obj.OrientationFormat,"Quaternion")
                        % Convert the input value to quartenion and plot
                        obj.visObj.Orientation = quaternion(orient);
                    else
                        obj.visObj.Orientation = orient;
                    end
                    drawnow limitrate;
                end
            end
        end

        function releaseImpl(obj)
            obj.visObj = [];
        end

        %% Validate input and properties
        function validateInputsImpl(obj, orient)
            if strcmpi(obj.OrientationFormat,"Quaternion")
                validateattributes(orient, {'double', 'single'}, ...
                    {'real', 'nonempty', '2d', 'nrows',1,'ncols', 4}, ...
                    '', 'Orientation in quaternion',1);
            else
                validateattributes(orient, {'double', 'single'}, ...
                    {'real', 'nonempty', '2d', 'nrows',3,'ncols', 3}, ...
                    '', 'Orientation in rotation matrix',1);
            end
        end
    end

    methods (Static, Access = protected)
        function simMode = getSimulateUsingImpl
            simMode = "Interpreted execution";
        end

        function flag = showSimulateUsingImpl
            flag = false;
        end
    end
end
