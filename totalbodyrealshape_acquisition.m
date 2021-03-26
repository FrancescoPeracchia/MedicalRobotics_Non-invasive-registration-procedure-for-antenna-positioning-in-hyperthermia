
close all
clc

%loading the body phantom points from the folder
a=load ('pts'); 
bodypts=a.bodyfinalshape; 

vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);


if (clientID>-1)
    disp('Connected to remote API server');

    %handle
    [~, referenceframeofinterest] = vrep.simxGetObjectHandle(clientID,'referenceframeworld',vrep.simx_opmode_blocking);
    [~, world] = vrep.simxGetObjectHandle(clientID,'mondo',vrep.simx_opmode_blocking);

    %Homogeneous transformation
    [~,worldtoref]=vrep.simxGetObjectPosition(clientID,world,referenceframeofinterest,vrep.simx_opmode_blocking);
    [~,worldtoreftor]=vrep.simxGetObjectOrientation(clientID,world,referenceframeofinterest,vrep.simx_opmode_blocking);
    R = eul2rotm(worldtoreftor);  
    Hbodypointstoprova(:,:)=[ R, worldtoref'; 0 0 0 1]*[bodypts;ones(1,size(bodypts,2))];
    Bodypointstoframe(:,:)=Hbodypointstoprova(1:3,:);
    
    xpoints=Bodypointstoframe(1,:);
    ypoints=Bodypointstoframe(2,:);
    zpoints=Bodypointstoframe(3,:);
else
    
    disp('Failed connecting to remote API server');

end
vrep.delete(); % call the destructor!
    
plot3(bodypts(1,:),bodypts(2,:),bodypts(3,:),'.')
xlabel('x')
ylabel('y')
zlabel('z')
legend('phantom points cloud');

%extracting points corresponding to thorax
Thorax_points_body=[];
j=1;
for i=1:size(Bodypointstoframe,2)
    if (Bodypointstoframe(1,i)<-0.2 || Bodypointstoframe(1,i)>0.17 || Bodypointstoframe(2,i)<-0.05  || Bodypointstoframe(2,i)>0.24 || Bodypointstoframe(3,i)>0.35|| Bodypointstoframe(3,i)<0.134 )
    else
        Thorax_points_body(:,j)=Bodypointstoframe(:,i);
        j=j+1;
    end
end
x_points_thorax=Thorax_points_body(1,:);
y_points_thorax=Thorax_points_body(2,:);
z_points_thorax=Thorax_points_body(3,:);

%extracting points corresponding to head
points_head=[];
j=1;
for i=1:size(Bodypointstoframe,2)
    if (Bodypointstoframe(1,i)<-0.1 || Bodypointstoframe(1,i)>0.1 || Bodypointstoframe(2,i)>0.405  ||Bodypointstoframe(2,i)<0.22  || Bodypointstoframe(3,i)<0.12 )
    else
        points_head(:,j)=Bodypointstoframe(:,i);
        j=j+1;
    end
end
x_points_head=points_head(1,:);
y_points_head=points_head(2,:);
z_points_head=points_head(3,:);



disp('Program ended');
