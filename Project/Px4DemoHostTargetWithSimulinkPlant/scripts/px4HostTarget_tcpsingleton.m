classdef px4HostTarget_tcpsingleton < handle
    %tcpsingleton Summary of this class goes here
    %   Detailed explanation goes here
    
    % Copyright 2020-2021 The MathWorks, Inc.
    
   methods(Static)
      % Concrete implementation.  See Singleton superclass.
      function tcpobj = getInstance(varargin)
         persistent uniqueInstance
         coder.extrinsic('tcpserver');
         if(nargin==0)
             if isempty(uniqueInstance)
                 % https://dev.px4.io/v1.10/en/simulation/#default-px4-mavlink-udp-ports
                 % The simulator's local TCP Port 4560 is used for
                 % communication with PX4. PX4 listens to this port, and simulators
                 % are expected to initiate the communication by broadcasting data to this port.
                 tcpobj = tcpserver("127.0.0.1", 4560);
                 uniqueInstance = tcpobj;
             else
                 tcpobj = uniqueInstance;
             end
         elseif(strcmp(varargin{1},'flush'))
             % This is required to kill the persistent variable tcp obj,
             % otherwise this port will be blocked
             % This is called from releaseImpl of tcpipread/tcpipwrite
             if ~isempty(uniqueInstance)
                 vars = whos;
                 vars = vars([vars.persistent]);
                 varName = {vars.name};
                 clear(varName{:});
             end
         end
      end
      
   end
   
end

