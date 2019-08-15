
% Download TC-720 Manual for complete command set at the website below
% https://tetech.com/product/tc-720/
%
% Download the correct FTDI VCP drivers for your system at the website below
% http://www.ftdichip.com/Drivers/VCP.htm



%% TeController: Te Tech TC-720 Thermoelectric controller
classdef TeController < handle
properties
    portName = 'com13';
    serialPort;
    timeout = 1; % in sec
    currSetTemp = 25;
    heatingPID;
    coolingPID;

    debugMode = false;  % print out debugging info
    noConnectionMode = false;  % operate without a serial connection

end
methods
    %% Constructor/Destructor/SerialConnection
    function obj = TeController(varargin)
        if ~isempty(varargin) && ischar(varargin{1})
            obj.portName = varargin{1};
        end
        % PID paramters for mouse paw stimulator (TE-31-1.0-2.0P)
        obj.heatingPID.BW = 3;
        obj.heatingPID.I  = 5;
        obj.heatingPID.D  = 10;
        obj.coolingPID.BW = 2.2;
        obj.coolingPID.I  = 11.17;
        obj.coolingPID.D  = 0;

        obj.connect();
        obj.setTemp(obj.currSetTemp);
    end

    function delete(obj)
        obj.disconnect();
        delete(obj.serialPort);
    end

    function connect(obj)
        if strcmp(obj.portName, 'noConn')
            obj.noConnectionMode = true;
        end
        if obj.noConnectionMode
           return
        end
        obj.serialPort = serial(obj.portName);
        obj.serialPort.BaudRate = 230400;
        fopen(obj.serialPort);
    end
    function disconnect(obj)
        if obj.noConnectionMode
           return
        end
       fclose(obj.serialPort);
    end

    %% Encode/Decode serial commands
    function response = receiveResponse(obj)
        response = [];
        startTime = clock;
        while (etime(startTime, clock) < obj.timeout)
            if obj.serialPort.BytesAvailable >= 8
                response = fscanf(obj.serialPort, '%s', 8);
                break
            end
            % pause(0.001) % this pause slows it down
        end
        if obj.debugMode
            fprintf('RESPONSE: "%s"\n', response);
        end
        if isempty(response)
            warning('Timeout: waiting for TC-720 to respond.');
            return
        elseif strcmp(response, '*XXXX60^')
            warning('Bad input to TC-720: either the command or checksum was incorrect.');
            return
        end
        response = hex2dec(response(2:5));
        if response >= 2^15
            response = response - 2^16;
        end
    end

    function response = sendMessage(obj, cmd, val)
        messageStart = '*';
        %messageEnd = char(13); % 13 => '\r'
        if ~( (numel(cmd) == 2) && (ischar(cmd)) )
            error('sendMessage: ''cmd'' must be a 2 character array')
        end
        messageCmd = lower(cmd);

        % get val in 2s-compliment hex:
        messageVal = dec2hex(2^16+val, 4);
        messageVal = lower(messageVal(end-3:end));

        % compute checksum
        messageChecksum = sum(int8(messageCmd)) + sum(int8(messageVal));
        messageChecksum = lower(dec2hex(mod(messageChecksum, 2^8), 2));

        % message = [messageStart, messageCmd, messageVal, messageChecksum, messageEnd];
        message = [messageStart, messageCmd, messageVal, messageChecksum];
        if obj.debugMode
            fprintf('MESSAGE: "%s"\n', message);
        end
        if obj.noConnectionMode
            response = 0;
        else
            fprintf(obj.serialPort, '%s\r', message, 'sync');
            response = obj.receiveResponse();
        end
    end

    %% Low-level user commands
    function setTemp(obj, temp)
        if (temp > obj.currSetTemp)
            obj.setPID(obj.heatingPID)
        else
            obj.setPID(obj.coolingPID)
        end
        obj.sendMessage('1c', round(temp*100));
        obj.currSetTemp = temp;
    end

    function temp = getTemp(obj)
       resp = obj.sendMessage('01', 0);
       temp = resp/100;
    end

    function turnOn(obj)
        obj.sendMessage('30', 1);
    end

    function turnOff(obj)
        obj.sendMessage('30', 0);
    end

    function setPID(obj, PIDSettings)
        obj.sendMessage('1d', PIDSettings.BW * 100); % Proportional Bandwidth
        obj.sendMessage('1e', PIDSettings.I * 100);  % Integral Gain
        obj.sendMessage('1f', PIDSettings.D * 100);  % Derivative Gain
    end


    %% High-Level user commands
    function runTemperatureSteps (obj, steps, stepDuration)
        % steps = [20, 15, 10, 30, 35, 40]; % in C
        % stepDuration = 30; % in sec

        nSteps = numel(steps);
        obj.setTemp(steps(1));

        % start temperature stimulus
        tempStimTimer = timer;
        tempStimTimer.ExecutionMode = 'fixedRate';
        tempStimTimer.Period = stepDuration;
        % steps(end+1) = steps(end);
        stim.steps = steps;
        stim.index = 0;
        stim.nSteps = nSteps;
        tempStimTimer.UserData = stim;
        tempStimTimer.TasksToExecute = nSteps + 1;
        tempStimTimer.TimerFcn = @(t, ~) obj.stepTimerFcn(t);
        tempStimTimer.StartFcn = @(t, ~) obj.stepStartFcn();
        tempStimTimer.StopFcn = @(t, ~) obj.stepStopFcn();
        start(tempStimTimer)
    end

    function stepTimerFcn(obj, timer)
        ud = timer.UserData;
        ud.index = ud.index + 1;
        set(timer,'UserData', ud);
        if (ud.index <= ud.nSteps)
            temp = ud.steps(ud.index);
            obj.setTemp(temp);
            if (timer.Period > 0.5)
                fprintf('Temperature Step: %g C\n', temp);
            end
        end
    end
    function stepStartFcn(obj)
        obj.turnOn();
        fprintf('Starting Temperature Steps.\n');
    end
    function stepStopFcn(obj)
        obj.turnOff();
        fprintf('Stopping Temperature Steps.\n');
    end

    function logTemperature (obj, samplePeriod, logDuration)
        nSamples = ceil(logDuration / samplePeriod);
        % start temperature stimulus
        tempLogTimer = timer;
        tempLogTimer.ExecutionMode = 'fixedRate';
        tempLogTimer.Period = samplePeriod;
        % steps(end+1) = steps(end);
        global temperatureLog,
        temperatureLog.time = zeros(1, nSamples)/0; % initialize to NaNs
        temperatureLog.temp = zeros(1, nSamples)/0; % initialize to NaNs
        temperatureLog.setPoint = zeros(1, nSamples)/0; % initialize to NaNs
        userData = [];
        userData.index = 0;
        tempLogTimer.UserData = userData;
        tempLogTimer.TasksToExecute = nSamples;
        tempLogTimer.TimerFcn = @(t, ~) obj.logTimerFcn(t);
        tempLogTimer.StartFcn = @(t, ~) obj.logStartFcn(t);
        tempLogTimer.StopFcn = @(t, ~) obj.logStopFcn();
        start(tempLogTimer)
    end

    function logTimerFcn(obj, timer)
        ud = timer.UserData;
        ud.index = ud.index + 1;
        set(timer,'UserData', ud);
        global temperatureLog
        temperatureLog.time(ud.index) = etime(clock, ud.startTime);
        temperatureLog.temp(ud.index) = obj.getTemp();
        temperatureLog.setPoint(ud.index) = obj.currSetTemp;
    end
    function logStartFcn(obj, timer)
        ud = timer.UserData;
        ud.startTime = clock;
        set(timer, 'UserData', ud);
        fprintf('Starting Temperature Log.\n');
    end
    function logStopFcn(obj)
        fprintf('Stopping Temperature Log.\n');
        global temperatureLog
        figure()
        plot(temperatureLog.time, [temperatureLog.temp; temperatureLog.setPoint])
    end


end % methods
end % class

