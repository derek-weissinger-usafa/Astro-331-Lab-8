classdef PreProcessor < matlab.System
    %PreProcessor  Process the IMU data for orientation estimation using
    %AHRS block.

    % The block accepts acceleration, angular rate and magnetic field as inputs.
    % The block outputs the processed IMU data suitable for the AHRS block.
    % The block subtracts compensation values specified by Magnetic field
    % offsets on the block mask from the magnetic field inputs and
    % align the axis to NED for the given sensor as per the indices and sign 
    % specified.


    %   Copyright 2022 The MathWorks, Inc.

    %#codegen
    properties(Nontunable)
        % Magnetic field offsets
        MagOffsets = [0,0,0];

        % Accelerometer axes parameters

        % Accelerometer X axis index
        AccelXAxisIndex (1,:) char {matlab.system.mustBeMember(AccelXAxisIndex,{'1', '2', '3'})} = '1';
        % Accelerometer X axis sign
        AccelXAxisSign (1,:) char {matlab.system.mustBeMember(AccelXAxisSign,{'+', '-'})} = '+';
        % Accelerometer Y axis index
        AccelYAxisIndex (1,:) char {matlab.system.mustBeMember(AccelYAxisIndex,{'1', '2', '3'})} = '2';
        % Accelerometer Y axis sign
        AccelYAxisSign (1,:) char {matlab.system.mustBeMember(AccelYAxisSign,{'+', '-'})} = '+';
        % Accelerometer Z axis index
        AccelZAxisIndex (1,:) char {matlab.system.mustBeMember(AccelZAxisIndex,{'1', '2', '3'})} = '3';
        % Accelerometer Z axis sign
        AccelZAxisSign (1,:) char {matlab.system.mustBeMember(AccelZAxisSign,{'+', '-'})} = '+';

        % Gyroscope axes parameters.

        % Gyroscope X axis index
        GyroXAxisIndex (1,:) char {matlab.system.mustBeMember(GyroXAxisIndex,{'1', '2', '3'})} = '1';
        % Gyroscope X axis sign
        GyroXAxisSign (1,:) char {matlab.system.mustBeMember(GyroXAxisSign,{'+', '-'})} = '+';
        % Gyroscope Y axis index
        GyroYAxisIndex (1,:) char {matlab.system.mustBeMember(GyroYAxisIndex,{'1', '2', '3'})} = '2';
        % Gyroscope Y axis sign
        GyroYAxisSign (1,:) char {matlab.system.mustBeMember(GyroYAxisSign,{'+', '-'})} = '+';
        % Gyroscope Z axis index
        GyroZAxisIndex (1,:) char {matlab.system.mustBeMember(GyroZAxisIndex,{'1', '2', '3'})} = '3';
        % Gyroscope Z axis sign
        GyroZAxisSign (1,:) char {matlab.system.mustBeMember(GyroZAxisSign,{'+', '-'})} = '+';

        % Magnetometer axes parameters.

        % Magnetometer X axis index
        MagXAxisIndex (1,:) char {matlab.system.mustBeMember(MagXAxisIndex,{'1', '2', '3'})} = '1';
        % Magnetometer X axis sign
        MagXAxisSign (1,:) char {matlab.system.mustBeMember(MagXAxisSign,{'+', '-'})} = '+';
        % Magnetometer Y axis index
        MagYAxisIndex  (1,:) char {matlab.system.mustBeMember(MagYAxisIndex,{'1', '2', '3'})} = '2';
        % Magnetometer Y axis sign
        MagYAxisSign (1,:) char {matlab.system.mustBeMember(MagYAxisSign,{'+', '-'})} = '+';
        % Magnetometer Z axis index
        MagZAxisIndex (1,:) char {matlab.system.mustBeMember(MagZAxisIndex,{'1', '2', '3'})} = '3';
        % Magnetometer Z axis sign
        MagZAxisSign (1,:) char {matlab.system.mustBeMember(MagZAxisSign,{'+', '-'})} = '+';
    end

    properties (Nontunable,Access = protected)
        GyroIndices = [1,2,3];
        GyroSigns = [1,1,1];
        AccelSigns = [1,1,1];
        AccelIndices = [1,2,3];
        MagSigns = [1,1,1];
        MagIndices = [1,2,3];
    end

    methods(Access = protected)
        function setupImpl(~)
        end

        function [accel,gyro,mag] = stepImpl(obj,accel,gyro,mag)
            % Compensate for hard iron distortion in magnetic field
            mag = mag-obj.MagOffsets;
            % Align sensor axis to NED Coordinates
            accel = alignAccelAxes(obj,accel);
            gyro = alignGyroAxes(obj,gyro);
            mag = alignMagAxes(obj,mag);
        end
    end

    methods
        function set.MagOffsets(obj,val)
            validateattributes(val, {'double', 'single'}, ...
                {'real', 'nonempty', '2d', 'nrows',1,'ncols', 3}, ...
                '', 'Magnetic field offsets');
            obj.MagOffsets = double(val);
        end
    end

    %% Output properties
    methods (Access=protected)
        function [IsAccelFixedSize,IsAngVelFixedSize,IsMagFixedSize] = isOutputFixedSizeImpl(~,~)
            IsAccelFixedSize = true;
            IsAngVelFixedSize = true;
            IsMagFixedSize = true;
        end

        function [IsAccelOutComplex,IsAngVelOutComplex,IsMagOutComplex] = isOutputComplexImpl(~)
            IsAccelOutComplex = false;
            IsAngVelOutComplex = false;
            IsMagOutComplex = false;
        end

        function [accelSize,angSize,magSize] = getOutputSizeImpl(~)
            accelSize = [1,3];
            angSize = [1,3];
            magSize  = [1,3];
        end

        function [accelDataType,angDataType,magDataType] = getOutputDataTypeImpl(~)
            accelDataType = 'double';
            angDataType = 'double';
            magDataType = 'double';
        end

        %% Validate input and properties
        function validateInputsImpl(obj, accelIn, gyroIn, magIn)
            % Validate that input data is of the type uint8
            validateattributes(accelIn, {'double', 'single'}, ...
                {'real', 'nonempty', '2d', 'nrows',1,'ncols', 3}, ...
                '', 'acceleration');
            validateattributes(gyroIn, {'double', 'single'}, ...
                {'real', 'nonempty', '2d', 'nrows',1,'ncols', 3}, ...
                '', 'angular rate');
            validateattributes(magIn, {'double', 'single'}, ...
                {'real', 'nonempty', '2d', 'nrows',1,'ncols', 3}, ...
                '', 'magnetic field');
            coder.extrinsic('PreProcessor.getIndices');

            % Get the Acceleration signs and indices
            [obj.AccelIndices ,obj.AccelSigns] = coder.const(@PreProcessor.getIndices,{obj.AccelXAxisIndex,obj.AccelYAxisIndex,obj.AccelZAxisIndex},{obj.AccelXAxisSign,obj.AccelYAxisSign,obj.AccelZAxisSign},'Accelerometer');

            % Get the AngularVelocity signs and indices
            [obj.GyroIndices, obj.GyroSigns] = coder.const(@PreProcessor.getIndices,{obj.GyroXAxisIndex,obj.GyroYAxisIndex,obj.GyroZAxisIndex},{obj.GyroXAxisSign,obj.GyroYAxisSign,obj.GyroZAxisSign},'Gyroscope');

            % Get the Magnetic field signs and indices
            [obj.MagIndices, obj.MagSigns] = coder.const(@PreProcessor.getIndices,{obj.MagXAxisIndex,obj.MagYAxisIndex,obj.MagZAxisIndex},{obj.MagXAxisSign,obj.MagYAxisSign,obj.MagZAxisSign},'Magnetometer');
        end

        function out = alignAccelAxes(obj,accel)
            out = [obj.AccelSigns(1), obj.AccelSigns(2), obj.AccelSigns(3)] .* ...
                accel(:, [obj.AccelIndices(1), obj.AccelIndices(2), obj.AccelIndices(3)]);
        end

        function out = alignGyroAxes(obj,gyro)
            out = [obj.GyroSigns(1), obj.GyroSigns(2), obj.GyroSigns(3)] .* ...
                gyro(:, [obj.GyroIndices(1), obj.GyroIndices(2), obj.GyroIndices(3)]);
        end

        function out = alignMagAxes(obj,mag)
            out = [obj.MagSigns(1), obj.MagSigns(2), obj.MagSigns(3)] .* ...
                mag(:, [obj.MagIndices(1), obj.MagIndices(2), obj.MagIndices(3)]);
        end
    end
    methods (Static, Access = protected)
        function simMode = getSimulateUsingImpl
            simMode = "Interpreted execution";
        end

        function flag = showSimulateUsingImpl
            flag = false;
        end

        function header = getHeaderImpl()
            txtString = ['Process the IMU data for orientation estimation using AHRS block.',newline,newline...
                ['The block accepts acceleration, angular rate, and magnetic field as inputs. ' ...
                'The block outputs the processed IMU data suitable for the AHRS block.'],newline,newline ...
                ['The block subtracts compensation values specified by ''Magnetic field offsets'' on the block mask from the magnetic ' ...
                'field inputs and align the axis to NED coordinates for the given sensor.'],newline,newline ...
                ['For your sensor, change the axis index and sign of acceleration,' ...
                ' angular rate and magnetic fields using the options provided below so that the sensor axes are aligned.' ...
                ' For example, to swap Accelerometer X-axis and Y-axis, ' ...
                'specify Accelerometer X-axis index as ''2'' and Accelerometer Y-axis index as ''1''. To invert values in an axis, ' ...
                'specify the axis sign as ''-''.';];];
            header = matlab.system.display.Header(mfilename('class'), 'Title',...
                'PreProcessor','Text',txtString);
        end
    end

    methods(Static, Hidden)
        function [indices, signs] = getIndices(maskIndices,maskSigns,sensor)
            signsOpt = {'+','-'};
            mulVal = [1,-1];
            signs = [mulVal(strcmp(signsOpt,maskSigns{1})),mulVal(strcmp(signsOpt,maskSigns{2})),mulVal(strcmp(signsOpt,maskSigns{3}))];
            indices = str2double({maskIndices{1},maskIndices{2},maskIndices{3}});
            if length(indices) ~= length(unique(indices))
                error(['Expected ',sensor,' indices to be unique. Ensure same index value is not specified for multiple axes on the ''PreProcessor'' block.']);
            end
        end
    end
end
