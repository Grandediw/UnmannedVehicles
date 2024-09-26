classdef px4HostTarget_TCPIP_Read < matlab.System
    % tcpipread 
    %

    % Copyright 2020-2021 The MathWorks, Inc.

    % Public, tunable properties
    properties
        %Data Length expected
        Length = 50
        %SampleTime Sample time
        SampleTime = -1;
    end

    properties(DiscreteState)
        PreviousValue
    end

    % Pre-computed constants
    properties(Access = private)
        %tcpip object
        t
    end
    
    methods
        
        % Constructor
        function obj = px4HostTarget_TCPIP_Read(varargin)
            coder.allowpcode('plain');
            % Support name-value pair arguments
            setProperties(obj,nargin,varargin{:});
        end
        
        function set.SampleTime(obj,newTime)
            coder.extrinsic('error');
            coder.extrinsic('message');
            coder.allowpcode('plain');
            if (isLocked(obj) && obj.SampleTime ~= newTime)
                error(message('svd:svd:SampleTimeNonTunable'))
            end
           
            isOk = isreal(newTime) && ...
                (all(all(isfinite(newTime))) || all(all(isinf(newTime)))) && ... %need to work all dimensions to scalar logical
                (numel(newTime) == 1 || numel(newTime) == 2);
            
            coder.internal.errorIf(~isOk,'svd:svd:InvalidSampleTimeNeedScalar');
            if ~isreal(newTime)
                newTime = real(newTime);
            end
            
            coder.internal.errorIf((newTime(1) < 0.0 && newTime(1) ~= -1.0),'svd:svd:InvalidSampleTimeNeedPositive');
            
            if numel(newTime) == 2
                coder.internal.errorIf((newTime(1) > 0.0 && newTime(2) >= newTime(1)),'svd:svd:InvalidSampleTimeNeedSmallerOffset');
                coder.internal.errorIf((newTime(1) == -1.0 && newTime(2) ~= 0.0),'svd:svd:InvalidSampleTimeNeedZeroOffset');
                coder.internal.errorIf((newTime(1) == 0.0 && newTime(2) ~= 1.0),'svd:svd:InvalidSampleTimeNeedOffsetOne');
            end
            
            obj.SampleTime = newTime;
        end
        
        function set.Length(obj,value)
            classes = {'numeric'};
            attributes = {'nonempty','nonnan','real','nonnegative','nonzero','scalar'};
            paramName = 'Expected Length';
            validateattributes(value,classes,attributes,'',paramName);
            obj.Length = value;
        end
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            
            obj.PreviousValue = uint8(zeros(obj.Length,1));
            
            % Accept a connection from any machine on port 30000.
            obj.t = px4HostTarget_tcpsingleton.getInstance();

        end

        function [status,data] = stepImpl(obj)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            % Get client data
            dataLen = obj.t.NumBytesAvailable;
            if(dataLen >= obj.Length )
                obj.PreviousValue = uint8(read(obj.t, obj.Length, "uint8")');
                status = uint8(1);
            else
                status = uint8(0);
            end
            data = obj.PreviousValue;
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
            obj.PreviousValue = uint8(zeros(obj.Length,1));
        end
        
        function releaseImpl(obj)
            obj.t = [];
            px4HostTarget_tcpsingleton.getInstance('flush');
        end
    end
    
    methods(Access=protected)
        function num = getNumOutputsImpl(~)
            % Define total number of outputs for system with optional
            % outputs
            num = 2;
        end

        function varargout = getOutputNamesImpl(~)
            % Return output port names for System block
            varargout{2} = 'Data';
            varargout{1} = 'Status';
        end
        
        function varargout = getOutputSizeImpl(obj)
            % Return size for each output port
            varargout{2} = [obj.Length 1];
            varargout{1} = [1 1];
        end

        function varargout = getOutputDataTypeImpl(~)
            varargout{1} = 'uint8';
            varargout{2} = 'uint8';
        end
        
        function varargout = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            varargout{1} = false;
            varargout{2} = false;
        end

        function varargout = isOutputFixedSizeImpl(~)
            % Return true for each output port with fixed size
            varargout{1} = true;
            varargout{2} = true;
        end
        
        function num = getNumInputsImpl(~)
            % Define total number of inputs for system with optional inputs
            num = 0;
        end
        
        function st = getSampleTimeImpl(obj)
            if isequal(obj.SampleTime, -1) || isequal(obj.SampleTime, [-1, 0])
                st = matlab.system.SampleTimeSpecification('Type', 'Inherited');
            elseif isequal(obj.SampleTime, [0, 1])
                st = matlab.system.SampleTimeSpecification('Type', 'Fixed In Minor Step');
            else
                if numel(obj.SampleTime) == 1
                    sampleTime = obj.SampleTime;
                    offset = 0;
                else
                    sampleTime = obj.SampleTime(1);
                    offset = obj.SampleTime(2);
                end
                st = matlab.system.SampleTimeSpecification('Type', 'Discrete', ...
                    'SampleTime', sampleTime, 'Offset', offset);
            end
        end
        
        function [sz,dt,cp] = getDiscreteStateSpecificationImpl(obj,name)
            if strcmp(name,'PreviousValue')
                sz = [obj.Length 1];
                dt = 'uint8';
                cp = false;
            else
                error(['Error: Incorrect State Name: ', name.']);
            end
        end
        
        function maskDisplayCmds = getMaskDisplayImpl(obj)
            num = 2;
            outport_label = [];
            outputs = getOutputNames(obj);
            for i = 1:num
                outport_label = [outport_label 'port_label(''output'',' num2str(i) ',''' outputs{i} ''');' newline]; %#ok<AGROW>
            end
            
            LocalIPPort = ['sprintf(''Port: %d'',' num2str(uint32(4560)) ')'];
            
            maskDisplayCmds = [ ...
                ['color(''white'');',newline]...
                ['plot([100,100,100,100]*1,[100,100,100,100]*1);',newline]...
                ['plot([100,100,100,100]*0,[100,100,100,100]*0);',newline]...
                ['color(''black'');',newline]...
                ['sppkgroot = strrep(codertarget.pixhawk.internal.getSpPkgRootDir(),''\'',''/'');',newline]...
                ['image(fullfile(sppkgroot,''resources'',''TCP.jpg''),''center'')',newline]...
                ['color(''black'');', newline] ...
                ['text(50,12,' LocalIPPort ' ,''horizontalAlignment'', ''center'');', newline], ...
                outport_label
                ];
            
            
        end
        
    end
    
    methods (Static, Access=protected)
        function simMode = getSimulateUsingImpl(~)
            simMode = 'Interpreted execution';
        end
        
        function isVisible = showSimulateUsingImpl
            isVisible = false;
        end
        
    end

    methods(Static, Access=protected)
        function header = getHeaderImpl()
            header = matlab.system.display.Header(mfilename('class'),...
                'ShowSourceLink', false, ...
                'Title','TCP Read from PX4 Host Target', ...
                'Text', ['This TCP connection is in Server mode with IP address 127.0.0.1 and Port 4560. The PX4 Host Target will connect to this Port' newline newline ...
                'The Data port outputs the values received as an [Nx1] array.' newline newline ...
                'The Status port outputs 0 when the available data size is less than the Data length expected parameter.']);
        end
              
    end
    
    methods
        function newTime = validateSampleTime(newTime)
            % Sample time must be a real scalar value or 2 element array.
            %#codegen
                       
            coder.allowpcode('plain');
            
            isOk = isreal(newTime) && ...
                (all(all(isfinite(newTime))) || all(all(isinf(newTime)))) && ... %need to work all dimensions to scalar logical
                (numel(newTime) == 1 || numel(newTime) == 2);
            
            coder.internal.errorIf(~isOk,'svd:svd:InvalidSampleTimeNeedScalar');
            if ~isreal(newTime)
                newTime = real(newTime);
            end
            
            coder.internal.errorIf((newTime(1) < 0.0 && newTime(1) ~= -1.0),'svd:svd:InvalidSampleTimeNeedPositive');
            
            if numel(newTime) == 2
                coder.internal.errorIf((newTime(1) > 0.0 && newTime(2) >= newTime(1)),'svd:svd:InvalidSampleTimeNeedSmallerOffset');
                coder.internal.errorIf((newTime(1) == -1.0 && newTime(2) ~= 0.0),'svd:svd:InvalidSampleTimeNeedZeroOffset');
                coder.internal.errorIf((newTime(1) == 0.0 && newTime(2) ~= 1.0),'svd:svd:InvalidSampleTimeNeedOffsetOne');
            end
            
        end
    end
end
