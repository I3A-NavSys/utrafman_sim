function slBusOut = Clock(msgIn, slBusOut, varargin)
%#codegen
%   Copyright 2021 The MathWorks, Inc.
    currentlength = length(slBusOut.Clock_);
    for iter=1:currentlength
        slBusOut.Clock_(iter) = bus_conv_fcns.ros.msgToBus.ros.Time(msgIn.Clock_(iter),slBusOut(1).Clock_(iter),varargin{:});
    end
    slBusOut.Clock_ = bus_conv_fcns.ros.msgToBus.ros.Time(msgIn.Clock_,slBusOut(1).Clock_,varargin{:});
end
