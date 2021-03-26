clear all
close all
clc

vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

jointreal_abdomen=[];

desired_joints_struct_abd=load('Abdomen_Variable_Joints.mat');
desired_joints_abdomen=desired_joints_struct_abd.abdomen_jointvariable;

K=4; %Proportional control
dt=0.01; %step time

if (clientID>-1)
    disp('Connected to remote API server');

    %handle
    [~, laser] = vrep.simxGetObjectHandle(clientID,'fast3DLaserScanner',vrep.simx_opmode_blocking);
    [~, referenceframeworld] = vrep.simxGetObjectHandle(clientID,'referenceframeworld',vrep.simx_opmode_blocking);
    [~, joint(1)] = vrep.simxGetObjectHandle(clientID,'Mico_joint1',vrep.simx_opmode_blocking);
    [~, joint(2)] = vrep.simxGetObjectHandle(clientID,'Mico_joint2',vrep.simx_opmode_blocking);
    [~, joint(3)] = vrep.simxGetObjectHandle(clientID,'Mico_joint3',vrep.simx_opmode_blocking);
    [~, joint(4)] = vrep.simxGetObjectHandle(clientID,'Mico_joint4',vrep.simx_opmode_blocking);
    [~, joint(5)] = vrep.simxGetObjectHandle(clientID,'Mico_joint5',vrep.simx_opmode_blocking);
    [~, joint(6)] = vrep.simxGetObjectHandle(clientID,'Mico_joint6',vrep.simx_opmode_blocking);
    
	
    [~, signalValue] = vrep.simxGetStringSignal(clientID,'measuredDataAtThisTime',vrep.simx_opmode_streaming);
    
    %inizialization
    [~]=vrep.simxSetJointPosition(clientID,joint(1),desired_joints_abdomen(1,1),vrep.simx_opmode_oneshot);
    [~]=vrep.simxSetJointPosition(clientID,joint(2),desired_joints_abdomen(2,1),vrep.simx_opmode_oneshot);
    [~]=vrep.simxSetJointPosition(clientID,joint(3),desired_joints_abdomen(3,1),vrep.simx_opmode_oneshot);
    [~]=vrep.simxSetJointPosition(clientID,joint(4),desired_joints_abdomen(4,1),vrep.simx_opmode_oneshot);
    [~]=vrep.simxSetJointPosition(clientID,joint(5),desired_joints_abdomen(5,1),vrep.simx_opmode_oneshot);
    [~]=vrep.simxSetJointPosition(clientID,joint(6),desired_joints_abdomen(6,1),vrep.simx_opmode_oneshot);
    for k=1:6
    [~,jointreal_abdomen(k,1)]=vrep.simxGetJointPosition(clientID,joint(k),vrep.simx_opmode_oneshot_wait);
    end
    
    
    for j=1:size(desired_joints_abdomen,2)
        counter=0;
        %Kinematic control (Proportional)
        while(abs(norm(jointreal_abdomen(:,j)-desired_joints_abdomen(:,j)))>0.0001)
            counter=counter+1;
            jointreal_abdomen(:,j)=jointreal_abdomen(:,j)+K*(-jointreal_abdomen(:,j)+desired_joints_abdomen(:,j))*dt;
            vrep.simxPauseCommunication(clientID,1);
            [~]=vrep.simxSetJointPosition(clientID,joint(1),jointreal_abdomen(1,j),vrep.simx_opmode_oneshot);
            [~]=vrep.simxSetJointPosition(clientID,joint(2),jointreal_abdomen(2,j),vrep.simx_opmode_oneshot);
            [~]=vrep.simxSetJointPosition(clientID,joint(3),jointreal_abdomen(3,j),vrep.simx_opmode_oneshot);
            [~]=vrep.simxSetJointPosition(clientID,joint(4),jointreal_abdomen(4,j),vrep.simx_opmode_oneshot);
            [~]=vrep.simxSetJointPosition(clientID,joint(5),jointreal_abdomen(5,j),vrep.simx_opmode_oneshot);
            [~]=vrep.simxSetJointPosition(clientID,joint(6),jointreal_abdomen(6,j),vrep.simx_opmode_oneshot);
            vrep.simxPauseCommunication(clientID,0);
            pause(0.01)
        end
        jointreal_abdomen(:,j+1)=jointreal_abdomen(:,j);
        
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
All_abdomen_points=[];
for j=1:size(desired_joints_abdomen,2)
    All_abdomen_points=[All_abdomen_points, Laserpointstoframe(:,:,j)];
end



%cleaning of thorax points acquired
Abdomen_points_cleaned=[];
j=1;
for i=1:size(All_abdomen_points,2)
    if (All_abdomen_points(1,i)<-0.17 || All_abdomen_points(1,i)>0.11 || All_abdomen_points(2,i)<0.03  || All_abdomen_points(2,i)>0.155 || All_abdomen_points(3,i)<0.145 )
    else
        Abdomen_points_cleaned(:,j)=All_abdomen_points(:,i);
        j=j+1;
    end
end
x_abdomen_model=Abdomen_points_cleaned(1,:);
y_abdomen_model=Abdomen_points_cleaned(2,:);
z_abdomen_model=Abdomen_points_cleaned(3,:);


   