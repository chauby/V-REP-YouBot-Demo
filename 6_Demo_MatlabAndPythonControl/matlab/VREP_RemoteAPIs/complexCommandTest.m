% This example illustrates how to execute complex commands from
% a remote API client. You can also use a similar construct for
% commands that are not directly supported by the remote API.
%
% Load the demo scene 'remoteApiCommandServerExample.ttt' in CoppeliaSim, then 
% start the simulation and run this program.
%
% IMPORTANT: for each successful call to simxStart, there
% should be a corresponding call to simxFinish at the end!

function complexCommandTest()

    disp('Program started');
    % sim=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
    sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    sim.simxFinish(-1); % just in case, close all opened connections
    clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);

    if (clientID>-1)
        disp('Connected to remote API server');
        
        % 1. First send a command to display a specific message in a dialog box:
        [res retInts retFloats retStrings retBuffer]=sim.simxCallScriptFunction(clientID,'remoteApiCommandServer',sim.sim_scripttype_childscript,'displayText_function',[],[],'Hello world!',[],sim.simx_opmode_blocking);
        if (res==sim.simx_return_ok)
            fprintf('Returned message: %s\n',retStrings);
        else
            fprintf('Remote function call failed\n');
        end

        % 2. Now create a dummy object at coordinate 0.1,0.2,0.3 with name 'MyDummyName':
        [res retInts retFloats retStrings retBuffer]=sim.simxCallScriptFunction(clientID,'remoteApiCommandServer',sim.sim_scripttype_childscript,'createDummy_function',[],[0.1 0.2 0.3],'MyDummyName',[],sim.simx_opmode_blocking);
        if (res==sim.simx_return_ok)
            fprintf('Dummy handle: %d\n',retInts(1));
        else
            fprintf('Remote function call failed\n');
        end

        % 3. Now send a code string to execute some random functions:
        code=['local octreeHandle=simCreateOctree(0.5,0,1)', char(10), ...
            'simInsertVoxelsIntoOctree(octreeHandle,0,{0.1,0.1,0.1},{255,0,255})', char(10), ...
            'return ''done'''];
        [res retInts retFloats retStrings retBuffer]=sim.simxCallScriptFunction(clientID,'remoteApiCommandServer',sim.sim_scripttype_childscript,'executeCode_function',[],[],code,[],sim.simx_opmode_blocking);
        if (res==sim.simx_return_ok)
            fprintf('Code execution returned: %s\n',retStrings);
        else
            fprintf('Remote function call failed\n');
        end

        % Now close the connection to CoppeliaSim:    
        sim.simxFinish(clientID);
    else
        disp('Failed connecting to remote API server');
    end
    sim.delete(); % call the destructor!
    
    disp('Program ended');
end
