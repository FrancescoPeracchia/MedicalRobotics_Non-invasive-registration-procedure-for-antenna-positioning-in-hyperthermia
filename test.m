clear all
close all
clc
 vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
    
    
     if (clientID>-1)
          disp('Connected to remote API server');
     else
         disp('Failed connecting to remote API server');
     end
     vrep.delete(); % call the destructor!
    
     disp('Program ended');