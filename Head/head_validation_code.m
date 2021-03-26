%function [fitresult, gof] = createFits(x_points_head, y_points_head, z_points_head, x_head_model, y_head_model, z_head_model)

%% Initialization.

% Initialize arrays to store fits and goodness-of-fit.
fitresult = cell( 2, 1 );
gof = struct( 'sse', cell( 2, 1 ), ...
    'rsquare', [], 'dfe', [], 'adjrsquare', [], 'rmse', [] );

%% Fit: 'real head fit'.
[xData, yData, zData] = prepareSurfaceData( x_points_head, y_points_head, z_points_head );

% Set up fittype and options.
ft = 'cubicinterp';

% Fit model to data.
f1h = fit( [xData, yData], zData, ft, 'Normalize', 'on' );

% Plot fit with data.
figure( 'Name', 'real head fit' );
h = plot( f1h );
legend( h, 'head real surface', 'z_points_head vs. x_points_head, y_points_head', 'Location', 'NorthEast', 'Interpreter', 'none' );
% Label axes
xlabel( 'x', 'Interpreter', 'none' );
ylabel( 'y', 'Interpreter', 'none' );
zlabel( 'z', 'Interpreter', 'none' );
grid on
view( 46.3, 29.8 );

%% Fit: 'model head fit'.
[xData, yData, zData] = prepareSurfaceData( x_head_model, y_head_model, z_head_model );

% Set up fittype and options.
ft = 'cubicinterp';

% Fit model to data.
f2h = fit( [xData, yData], zData, ft, 'Normalize', 'on' );

% Plot fit with data.
figure( 'Name', 'model head fit' );
h = plot( f2h) ;
legend( h, 'model head fit', 'z_head_model vs. x_head_model, y_head_model', 'Location', 'NorthEast', 'Interpreter', 'none' );
% Label axes
xlabel( 'x', 'Interpreter', 'none' );
ylabel( 'y', 'Interpreter', 'none' );
zlabel( 'z', 'Interpreter', 'none' );
grid on
view( -78.0, 0.6 );

%% Computing the goodness of modelled surface


[x_grid_head,y_grid_head] = meshgrid(-0.07:0.001:0.055,0.23:0.001:0.4);
headreal  = f1h(x_grid_head,y_grid_head);
headmodel = f2h(x_grid_head,y_grid_head);
diff=headreal-headmodel;

%Calculating the RSS by excluding NaNs
indiciNaN=isnan(diff);
maxz=max(headreal,[],'all');
minz=min(headreal,[],'all');
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
surf(x_grid_head,y_grid_head,diff)
xlabel('x')
ylabel('y')
zlabel('z')
legend('residuals')



