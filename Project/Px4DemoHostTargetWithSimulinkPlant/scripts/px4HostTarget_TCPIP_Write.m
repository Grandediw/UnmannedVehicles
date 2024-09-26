classdef px4HostTarget_TCPIP_Write < matlab.System

    % tcpipwrite Add summary here
    %

    % Copyright 2020-2021 The MathWorks, Inc.

    % Public, tunable properties
    properties

    end

    properties(DiscreteState)

    end

    % Pre-computed constants
    properties(Access = private)
        %tcpip object
        t
    end
    
    methods
        
        % Constructor
        function obj = px4HostTarget_TCPIP_Write(varargin)
            coder.allowpcode('plain');
            % Support name-value pair arguments
            setProperties(obj,nargin,varargin{:});
        end
        
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.t = px4HostTarget_tcpsingleton.getInstance();
            %Open a connection. This will not return until a connection is received.
        end

        function stepImpl(obj,input)
            write(obj.t,input, "uint8");
        end

        function resetImpl(~)
            % Initialize / reset discrete-state properties
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
            num = 0;
        end
        
        function num = getNumInputsImpl(~)
            % Define total number of inputs for system with optional inputs
            num = 1;
        end
        
        function validateInputsImpl(~,in)
            if ~isa(in,'uint8')
                error('Input must be uint8');
            end
        end
        
        function maskDisplayCmds = getMaskDisplayImpl(~)
            
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
                ];

        end
        
    end
    
    methods(Static, Access=protected)
        function header = getHeaderImpl()
            header = matlab.system.display.Header(mfilename('class'),...
                'ShowSourceLink', false, ...
                'Title','TCP Write to PX4 Host Target', ...
                'Text', ['This TCP connection is in Server mode with IP address 127.0.0.1 and Port 4560. The PX4 Host Target will connect to this Port' newline newline ...
                'The block accepts a 1-D array of type uint8, int8, uint16, int16, uint32, int32, boolean, single-precision or double-precision.']);
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
    
end
