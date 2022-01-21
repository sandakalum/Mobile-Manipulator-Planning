
%**************************************************************************
%kni_matlab.m
%This is a demo program to test the KNI_Wrapper.dll
%
%Copyright (C) Neuronics AG
%Tino Perucchi, 2008
%**************************************************************************

%%
function kni_matlab(configfile, ipAddress)

ENC_TOLERANCE = 10;
DEFAULT_ACCELERATION = 2;
DEFAULT_SPEED = 100;
POSITIONAL_OFFSET = 10000;

if nargin<2
    disp('usage: kni_wrapper CONFIGFILE IP_ADDR')
    return
end

%clear command window
clc

%unload 'kni'
unloadKniLib()


disp('-------------------------------------------')
%load KNI_Wrapper.dll
loadKniLib()
disp('Starting to call KNI interfaces.')
disp('Interfaces in brackets[] are only being called implicitly.')
disp('-------------------------------------------')

% inititiate Katana
try
    fprintf('Inititate Katana.. ')
    ret = calllib('kni','initKatana',char(configfile),char(ipAddress));
    if ret ~= 1
        fprintf('error!\n')
        unloadKniLib()
        return
    end
        
    fprintf('done.\n')
catch
    err= lasterr;
    disp(err)
    unloadKniLib()
    return
end


%perform a sequence of KNI calls
try
    %calibrate Katana
    fprintf('Calibrate Katana.. ')
    calllib('kni','calibrate',0);
    fprintf('done.\n ')
    
    %*****************
    %Stage 1
    %*****************
    fprintf('\n*********\n STAGE 1 \n*********\n')
    printInterface('\t [moveMot(int axis, int enc, int speed, int accel)],\n\t getEncoder(int axis),\n\t [waitForMot(int axis, int targetpos=0, int tolerance=0)]\n\t moveMotAndWait(int axis, int targetpos, int tolerance=0)');
    nbMotors = calllib('kni','getNumberOfMotors');
    fprintf('Number of motors found: %d\n',nbMotors);
    for i = 1:nbMotors;
        encoder_p = libpointer('int32Ptr', 0);
        calllib('kni','getEncoder',i,encoder_p);
        enc(i)=get(encoder_p,'Value');
        
        if enc(i)>0
            targetenc(i) = enc(i) - POSITIONAL_OFFSET;
        else
            targetenc(i) = enc(i) + POSITIONAL_OFFSET;
        end
        
        fprintf('Moving motor %d.. ',i);
        calllib('kni','moveMotAndWait',i, targetenc(i), ENC_TOLERANCE);
        calllib('kni','moveMotAndWait',i, enc(i), ENC_TOLERANCE);
        fprintf('done.\n')
    end

    
    %*****************
    %Stage 2
    %*****************
    fprintf('\n*********\n STAGE 2 \n*********\n')
    printInterface('\t [motorOff(int axis)],\n\t [motorOn(int axis)],\n\t allMotorsOff()\n\t allMotorsOn()')
     calllib('kni','allMotorsOff');
     calllib('kni','allMotorsOn');
     calllib('kni','calibrate',0);

    
    %****************
    %Stage 3
    %****************
    fprintf('\n*********\n STAGE 3 \n*********\n')
    printInterface('\t moveToPosEnc(int enc1, int enc2, int enc3, int enc4, int enc5, int enc6, int velocity, int acceleration, int tolerance)');
    for i = 1:nbMotors
        fprintf('startEncoder: %d \t targetEncoder: %d\n',enc(i),targetenc(i));
    end

    fprintf('\nMoving to targetEncoder.. ');
    calllib('kni','moveToPosEnc', targetenc(1),  targetenc(2),  targetenc(3),  targetenc(4), targetenc(5), targetenc(6), DEFAULT_SPEED, DEFAULT_ACCELERATION, ENC_TOLERANCE, true);
    fprintf('done.\n')
    
    fprintf('Moving to startEncoder.. ');
    calllib('kni','moveToPosEnc', enc(1),  enc(2),  enc(3),  enc(4), enc(5), enc(6), DEFAULT_SPEED, DEFAULT_ACCELERATION, ENC_TOLERANCE, true);
    fprintf('done.\n')
    
    
    %****************
    %Stage4
    %****************
    fprintf('\n*********\n STAGE 4 \n*********\n')
    printInterface('\tgetPosition(struct TPos &pos)\n\tmoveToPos(struct TPos pos)\n\tmoveToPosLin(struct TPos startPos, struct TPos targetPos)');

    %create a MATLAB structure corresponding to the C struct 'TPos'
    Tpos.X = 0;
    Tpos.Y = 0;
    Tpos.Z = 0;
    Tpos.phi = 0;
    Tpos.theta = 0;
    Tpos.psi = 0;

    %create a pointer to the 'TPos' struct defined in kni_wrapper.h
    currentPosition = libstruct('TPos',Tpos);
    targetPosition = libstruct('TPos',Tpos);
    targetPosition2 = libstruct('TPos',Tpos);

    %Read the coordinates point A
    calllib('kni','getPosition', currentPosition);
    fprintf('Current_position:\n');
    get(currentPosition)
    
    %Move to point B (encoders)
    calllib('kni','moveToPosEnc', targetenc(1),  targetenc(2), targetenc(3),  targetenc(4), targetenc(5), targetenc(6), DEFAULT_SPEED, DEFAULT_ACCELERATION, ENC_TOLERANCE, true);
    
    %Read the coordinates of point B
    calllib('kni','getPosition', targetPosition);
    fprintf('Target_position:\n');
    get(targetPosition)
    
    %Move to point A (encoders)
    calllib('kni','moveToPosEnc', enc(1),  enc(2),  enc(3),  enc(4), enc(5), enc(6), DEFAULT_SPEED, DEFAULT_ACCELERATION, ENC_TOLERANCE, true);
    
    fprintf('Moving to targetPosition.. ')
    
    %Move to point B (coordinates)
    calllib('kni','moveToPos', targetPosition, 100, DEFAULT_ACCELERATION);
    fprintf('done.\n')
    
    %Some linear movements (coordinates)..
    calllib('kni','moveToPosEnc', 6355, -13513, -27931, 8500, 19832, 30420, DEFAULT_SPEED, DEFAULT_ACCELERATION, ENC_TOLERANCE, true);
    calllib('kni','getPosition', targetPosition2);
    targetPosition2.Y = targetPosition2.Y - 160;
    fprintf('Linear movements.. ')
    calllib('kni','moveToPosLin', targetPosition2, 100, DEFAULT_ACCELERATION);
    targetPosition2.X = targetPosition2.X - 300;
    calllib('kni','moveToPosLin', targetPosition2, 100, DEFAULT_ACCELERATION);
    targetPosition2.Z = targetPosition2.Z + 150;
    calllib('kni','moveToPosLin', targetPosition2, 100, DEFAULT_ACCELERATION);
    fprintf('done.\n')
    
    %Move to point A (encoder)
    fprintf('Go back home.. ')
    calllib('kni','moveToPosEnc', enc(1),  enc(2),  enc(3),  enc(4), enc(5), enc(6), DEFAULT_SPEED, DEFAULT_ACCELERATION, ENC_TOLERANCE, true);
    fprintf('done.\n\n')
    
    clear currentPosition
    clear targetPosition
    clear targetPosition2

catch
    err= lasterr;
    disp(err)
    unloadKniLib();
    return
end

% Unload kni
unloadKniLib();
disp('Bye-bye!')

end


%%
% this function load the library 'KNI_Wrapper.dll'
function loadKniLib()

%unload 'kni'
unloadKniLib();

loadlibrary('..\..\lib\win32\KNI_Wrapper.dll',...
    '..\..\include\kni_wrapper\kni_wrapper.h','alias','kni');
disp('KNI library successfully loaded.')

%display the names of all the functions in the library
%libfunctionsview('kni')
end

%%
% this function unload the library 'KNI_Wrapper.dll'
function unloadKniLib()

%unload 'kni'
if libisloaded('kni')
    unloadlibrary('kni')
    disp('KNI library unloaded.')
end

end

%%
function printInterface(string)
string = strcat('\nCalling interface:\n',string,'\n\n');
fprintf(string)
end






