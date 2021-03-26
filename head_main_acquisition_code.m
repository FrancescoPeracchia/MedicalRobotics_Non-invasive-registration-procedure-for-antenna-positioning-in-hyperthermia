clear all
close all
clc

vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

jointrealhead=[];
%t = vrep.simxGetLastCmdTime(clientID) / 1000.0;  % get current simulation time
desired_joints_struct_=load('Head_Variable_Joints.mat');
head_jointvariable=desired_joints_struct_.head_jointvariable;

K=4; %Proportional control
dt=0.01; %step time

if (clientID>-1)
    disp('Connected to remote API server');
    
    %handle
    [~, laser] = vrep.simxGetObjectHandle(clientID,'fast3DLaserScanner',vrep.simx_opmode_blocking);
    [~, table] = vrep.simxGetObjectHandle(clientID,'customizableTable#0',vrep.simx_opmode_blocking);
    [~, joint(1)] = vrep.simxGetObjectHandle(clientID,'Mico_joint1',vrep.simx_opmode_blocking);
    [~, joint(2)] = vrep.simxGetObjectHandle(clientID,'Mico_joint2',vrep.simx_opmode_blocking);
    [~, joint(3)] = vrep.simxGetObjectHandle(clientID,'Mico_joint3',vrep.simx_opmode_blocking);
    [~, joint(4)] = vrep.simxGetObjectHandle(clientID,'Mico_joint4',vrep.simx_opmode_blocking);
    [~, joint(5)] = vrep.simxGetObjectHandle(clientID,'Mico_joint5',vrep.simx_opmode_blocking);
    [~, joint(6)] = vrep.simxGetObjectHandle(clientID,'Mico_joint6',vrep.simx_opmode_blocking);
    
    [~, referenceframeworld] = vrep.simxGetObjectHandle(clientID,'referenceframeworld',vrep.simx_opmode_blocking);
	[~, signalValue] = vrep.simxGetStringSignal(clientID,'measuredDataAtThisTime',vrep.simx_opmode_streaming);
    
	
    %[~,startpoint]=vrep.simxGetObjectPosition(clientID,start,referenceframeworld,vrep.simx_opmode_blocking);
    [~]=vrep.simxSetObjectPosition(clientID,table,-1,[-0.33200,-0.325,1.0931], vrep.simx_opmode_oneshot);
    pause(1)
            
    %inizialization
    [~]=vrep.simxSetJointPosition(clientID,joint(1),head_jointvariable(1,1),vrep.simx_opmode_oneshot);
    [~]=vrep.simxSetJointPosition(clientID,joint(2),head_jointvariable(2,1),vrep.simx_opmode_oneshot);
    [~]=vrep.simxSetJointPosition(clientID,joint(3),head_jointvariable(3,1),vrep.simx_opmode_oneshot);
    [~]=vrep.simxSetJointPosition(clientID,joint(4),head_jointvariable(4,1),vrep.simx_opmode_oneshot);
    [~]=vrep.simxSetJointPosition(clientID,joint(5),head_jointvariable(5,1),vrep.simx_opmode_oneshot);
    [~]=vrep.simxSetJointPosition(clientID,joint(6),head_jointvariable(6,1),vrep.simx_opmode_oneshot);
   
    for k=1:6
    [~,jointrealhead(k,1)]=vrep.simxGetJointPosition(clientID,joint(k),vrep.simx_opmode_oneshot_wait);
    end
    
    for j=1:size(head_jointvariable,2)
        counter=0;
        %Kinematic control (proportional)
        while(abs(norm(jointrealhead(:,j)-head_jointvariable(:,j)))>0.001)
            counter=counter+1
            jointrealhead(:,j)=jointrealhead(:,j)+K*(-jointrealhead(:,j)+head_jointvariable(:,j))*dt;
            vrep.simxPauseCommunication(clientID,1);
            [~]=vrep.simxSetJointPosition(clientID,joint(1),jointrealhead(1,j),vrep.simx_opmode_oneshot);
            [~]=vrep.simxSetJointPosition(clientID,joint(2),jointrealhead(2,j),vrep.simx_opmode_oneshot);
            [~]=vrep.simxSetJointPosition(clientID,joint(3),jointrealhead(3,j),vrep.simx_opmode_oneshot);
            [~]=vrep.simxSetJointPosition(clientID,joint(4),jointrealhead(4,j),vrep.simx_opmode_oneshot);
            [~]=vrep.simxSetJointPosition(clientID,joint(5),jointrealhead(5,j),vrep.simx_opmode_oneshot);
            [~]=vrep.simxSetJointPosition(clientID,joint(6),jointrealhead(6,j),vrep.simx_opmode_oneshot);
            vrep.simxPauseCommunication(clientID,0);
            pause(0.01)
      end  
      jointrealhead(:,j+1)=jointrealhead(:,j);
      
      
      %Acquiring points
      i=1;
      while(i<3)
            [returnCode, signalValue] = vrep.simxGetStringSignal(clientID,'measuredDataAtThisTime',vrep.simx_opmode_buffer);
            data=vrep.simxUnpackFloats(signalValue);
            pause(0.01);
            i=i+1;
      end
  
      %unwrap points
      X_to_Laser=data(1:3:end-2); 
      Y_to_Laser=data(2:3:end-1);
      Z_to_Laser=data(3:3:end);
      Imm_To_Laser= [X_to_Laser;Y_to_Laser;Z_to_Laser];
        
      %Homogeneous transformation
      [~,laserpositiontoframe]=vrep.simxGetObjectPosition(clientID,laser,referenceframeworld,vrep.simx_opmode_blocking);
      [~,laserorientationntoframe]=vrep.simxGetObjectOrientation(clientID,laser,referenceframeworld,vrep.simx_opmode_blocking);
      R = eul2rotm(laserorientationntoframe,'XYZ');
      HLaserpointstoframe(:,:,j)=[ R, laserpositiontoframe'; 0 0 0 1]*[Imm_To_Laser;ones(1,size(Imm_To_Laser,2))];
      Laserpointstoframe(:,:,j)=HLaserpointstoframe(1:3,:,j);
                
    end
else
    disp('Failed connecting to remote API server');
end
vrep.delete(); % call the destructor!
    
disp('Program ended');

%Collecting all points
All_points_head=[];
for j=1:size(head_jointvariable,2)
    All_points_head=[All_points_head, Laserpointstoframe(:,:,j)];
end


%cleaning of head points acquired
Head_points_cleaned=[];
j=1;
for i=1:size(All_points_head,2)
    if (All_points_head(1,i)<-0.1 || All_points_head(1,i)>0.1 || All_points_head(2,i)>0.42  || All_points_head(2,i)<0.22 || All_points_head(3,i)<0.1 )
    else
       Head_points_cleaned(:,j)=All_points_head(:,i);
      j=j+1;
    end
end
x_head_model=Head_points_cleaned(1,:);
y_head_model=Head_points_cleaned(2,:);
z_head_model=Head_points_cleaned(3,:);


   