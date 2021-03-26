close all
clc

[xData, yData, zData] = prepareSurfaceData( x_abdomen_model, y_abdomen_model, z_abdomen_model );

% Set up fittype and options.
ft = 'cubicinterp';

% Fit model to data.
[f1a, gof] = fit( [xData, yData], zData, ft, 'Normalize', 'on' );

% Plot fit with data.
figure( 'Name', 'abdomen_fit' );
h = plot( f1a )
legend( h, 'abdomen model surface', 'z_abdomen_model vs. x_abdomen_model, y_abdomen_model', 'Location', 'NorthEast', 'Interpreter', 'none' );
% Label axes
xlabel( 'x', 'Interpreter', 'none' );
ylabel( 'y', 'Interpreter', 'none' );
zlabel( 'z', 'Interpreter', 'none' );
grid on
view( 66.7, 48.8 );

[x_grid_abd,y_grid_abd] = meshgrid(-0.168:0.001:0.009,0.06:0.001:0.152);
abdomenmodel  = f1a(x_grid_abd,y_grid_abd);
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID>-1)
    disp('Connected to remote API server');

    %handle
    [~, referenceframeworld] = vrep.simxGetObjectHandle(clientID,'referenceframeworld',vrep.simx_opmode_blocking);
    [~,referencetoworld]=vrep.simxGetObjectPosition(clientID,referenceframeworld,-1,vrep.simx_opmode_blocking);
    %[~,referencetoworldor]=vrep.simxGetObjectOrientation(clientID,referenceframeworld,-1,vrep.simx_opmode_blocking);
    %R = eul2rotm(referencetoworldor,'XYZ');
    
    %referring everthing to the world for a correct export
    x_grid_abd_toworld=x_grid_abd+referencetoworld(1);
    y_grid_abd_toworld=y_grid_abd+referencetoworld(2);
    abdomenmodel_to_world=abdomenmodel+referencetoworld(3);
else
    disp('fail');
end

x_grid_abd_toworld=double(x_grid_abd_toworld);
y_grid_abd_toworld=double(y_grid_abd_toworld);
abdomenmodel_to_world=double(abdomenmodel_to_world);

figure
surf(x_grid_abd_toworld,y_grid_abd_toworld,abdomenmodel_to_world);
xlabel('x');
ylabel('y');
zlabel('z');
legend('abdomen');

%converting surface to stl object for importin it into v-rep 
%stlwrite('finaladdome1.stl',x_grid_abd_toworld,y_grid_abd_toworld,abdomenmodel_to_world,'mode','ascii');



