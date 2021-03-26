%function [fitresult, gof] = createFits(x_points_thorax, y_points_thorax, z_points_thorax, x_thorax_model, y_thorax_model, z_thorax_model)


%% Initialization.

% Initialize arrays to store fits and goodness-of-fit.
fitresult = cell( 2, 1 );
gof = struct( 'sse', cell( 2, 1 ), ...
    'rsquare', [], 'dfe', [], 'adjrsquare', [], 'rmse', [] );

%% Fit: 'thorax_real_fit'.
[xData, yData, zData] = prepareSurfaceData( x_points_thorax, y_points_thorax, z_points_thorax );

% Set up fittype and options.
ft = 'cubicinterp';

% Fit model to data.
f1 = fit( [xData, yData], zData, ft, 'Normalize', 'on' );

% Plot fit with data.
figure( 'Name', 'thorax_real_fit' );
h = plot( f1 );
legend( h, 'thorax real surface', 'z_points_thorax vs. x_points_thorax, y_points_thorax', 'Location', 'NorthEast', 'Interpreter', 'none' );
% Label axes
xlabel( 'x', 'Interpreter', 'none' );
ylabel( 'y', 'Interpreter', 'none' );
zlabel( 'z', 'Interpreter', 'none' );
grid on

%% Fit: 'thorax_model_fit'.
[xData, yData, zData] = prepareSurfaceData( x_thorax_model, y_thorax_model, z_thorax_model );

% Set up fittype and options.
ft = 'cubicinterp';

% Fit model to data.
f2 = fit( [xData, yData], zData, ft, 'Normalize', 'on' );

% Plot fit with data.
figure( 'Name', 'thorax_model_fit' );
h = plot( f2 );
legend( h, 'thorax model surface', 'z_thorax_model vs. x_thorax_model, y_thorax_model', 'Location', 'NorthEast', 'Interpreter', 'none' );
% Label axes
xlabel( 'x', 'Interpreter', 'none' );
ylabel( 'y', 'Interpreter', 'none' );
zlabel( 'z', 'Interpreter', 'none' );
grid on
view( -55.7, 22.3 );

%% Computing the goodness of modelled surface

[x_grid,y_grid] = meshgrid(-0.15:0.001:0.14,0:0.001:0.2);
bodyreal  = f1(x_grid,y_grid);
bodymodel = f2(x_grid,y_grid);
diff=bodyreal-bodymodel;

%Calculating the RSS by excluding NaNs
indiciNaN=isnan(diff);
maxz=max(bodyreal,[],'all');
minz=min(bodyreal,[],'all');
s=0;
for i=1:size(diff,1)
    for j=1:size(diff,2)
        if indiciNaN(i,j)==0
        s=s+((diff(i,j))^2); 
        end
    end
end    
RSS=sqrt(1/size(diff,1)/size(diff,2)*s);

fprintf('The residual sum of squared is %.8f',RSS);

%plotting the residuals
figure
surf(x_grid,y_grid,diff)
xlabel('x')
ylabel('y')
zlabel('z')
legend('residuals')
