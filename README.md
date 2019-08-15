# Te-Peltier-Controller
Matlab interface for a TE Technologies TC-720 Thermoelectric Controller

## Setup
Set up the TC-720 as described in the manual and connect to your computer using the USB cable.

Be sure to download and install the correct FTDI VCP drivers for your system from http://www.ftdichip.com/Drivers/VCP.htm

This code mostly uses the default settings on the TC-720. In only changes the PID parameters and the temperature set point. Depending on your setup, you may need to adjust other paramteres, e.g., the type of thermister used.

## Usage Examples
1) Open a connection to the TC-720 on com port 3:
```matlab
tc = TeController('com3');
```

2) Start a sequence of temperature steps:
```matlab
TemperatureSteps = [25, 30, 35, 40, 25, 15, 10, 5, 25]; % in C
StepDuration = 30; % in sec
tc.runTemperatureSteps(TemperatureSteps, StepDuration); % runs in background
```

3) Log temperature measurements while delivering a sequence of temperature steps:
```matlab
SamplePeriod = 0.1; % in sec
LogDuration = 30; % in sec
tc.logTemperature(SamplePeriod, LogDuration); % runs in background

TemperatureSteps = [25, 30, 35, 40, 25, 15, 10, 5, 25]; % in C
StepDuration = 30; % in sec
tc.runTemperatureSteps(TemperatureSteps, StepDuration); % runs in background

%% When logging is done:
global temperatureLog
figure()
plot(temperatureLog.time, [temperatureLog.temp; temperatureLog.setPoint])
```


