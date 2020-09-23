% [Project]        [Example] Trajectory Tracking via Sliding Variable
% [Author]         Moses C. Nah
% [Creation Date]  Sunday, September 20th, 2020
% [Emails]         Moses C. Nah   : mosesnah@mit.edu

%% (--) INITIALIZATION
clear all; close all; clc;

cd( fileparts( matlab.desktop.editor.getActiveFilename ) );                % Setting the current directory as the folder where this "main.m" script is located

myFigureConfig(     'fontsize',  20, ...
                   'lineWidth',   5, ...
                  'markerSize',  25 );               
c  = myColor();                                                            % Setting the color as global.

%% (1A) n-Link Manipulator Example #1.
% Tracking a circular trajectory with constant angular acceleration.
clc
l  = 1  * ones( 1, 9 );  
qi = 0.05 * ones( 1, 9 );
N = length( qi );                                                          % Degrees of freedom of the robot.

robot = myPlanarRobot( l );
robot.initializeAnimation( qi )

tmpLim = sum( l ) + 2;                                                     % 2 As spacing
set( robot.hAxes( 1 ),   'XLim',   [ -tmpLim , tmpLim ] , ...                  
                         'YLim',   [ -tmpLim , tmpLim ] )


% [PARAMETER SETTINGS]
% Setting the parameters for sliding-variable position tracking.
% The equation of desired trajectory's position + velocity w.r.t. time.

syms t
idx_c = 4;

if idx_c == 1
    % [Constraint Type 1] - Circle, xd with constant angular acceleration
    R  = 4;                                                                % Radius of the Desired Trajectory
    xd = [R * cos( 1/2 * t.^2 ), R * sin( 1/2 * t.^2 )];                   % Desired Trajectory Position w.r.t. time
    vd = diff( xd, t );                                                    % Desired Trajectory Velocity w.r.t. time 
    xd = matlabFunction( xd ); vd = matlabFunction( vd );                  % Converting symbolic expression as function handle.          

    % Drawing the circle to track.                   
    ttmp = 0 : 0.001 : 2*3.141592;
    plot( R * cos( ttmp ), R * sin( ttmp ), 'parent', robot.hAxes(1), ...
                'linewidth', 3, 'linestyle', '--', 'color', c.blue_sky )                    
    
elseif idx_c == 2
    % [Constraint Type 2] - Straight Line, oscillatory movement.
    pi = [ 2,-3 ];
    pf = [ 2, 3 ];
    xd = ( pi + pf )/2 + ( pi - pf )/2 * cos( 2 * t );                     % Desired Trajectory Position w.r.t. time
    vd = diff( xd, t );                                                    % Desired Trajectory Velocity w.r.t. time 
    xd = matlabFunction( xd ); vd = matlabFunction( vd );                  % Converting symbolic expression as function handle.          
    
    plot( [ pi( 1 ), pf( 1 ) ], [ pi( 2 ), pf( 2 ) ], 'linewidth', 3, 'linestyle', '--', 'color', c.blue_sky )                        
    
elseif idx_c == 3
    % [Constraint Type 3] - Infinity Sign.
    p0 = [4,0];
    xw = 1; yw = 3;
                                                                           % Radius of the Desired Trajectory
    xd  = [p0(1) + xw * cos( t ), p0(2) + yw * sin( 2 * t )];              % Desired Trajectory Position w.r.t. time
    vd  = diff( xd, t );                                                   % Desired Trajectory Velocity w.r.t. time 
    xd  = matlabFunction( xd ); vd = matlabFunction( vd );                 % Converting symbolic expression as function handle.          
    
    ttmp = 0 : 0.001 : 2*3.141592;    
    
    tmp = arrayfun( xd, ttmp , 'UniformOutput', false); tmp = cell2mat( tmp' );

    plot( tmp( :,1 ), tmp( :,2 ), 'parent', robot.hAxes(1), ...
                'linewidth', 3, 'linestyle', '--', 'color', c.blue_sky )       
            
elseif idx_c == 4
    % [Constraint Type 4] - Heart Curve
    xd  = 0.4 * [16 * sin(t)^3, 13 * cos(t) - 5 * cos(2*t) - 2* cos(3*t) - cos(4*t)];
    vd  = diff( xd, t );                                                   % Desired Trajectory Velocity w.r.t. time 
    xd  = matlabFunction( xd ); vd = matlabFunction( vd );                 % Converting symbolic expression as function handle.          

    ttmp = 0 : 0.001 : 2*3.141592;    
    
    tmp = arrayfun( xd, ttmp , 'UniformOutput', false); tmp = cell2mat( tmp' );

    plot( tmp( :,1 ), tmp( :,2 ), ...
                'linewidth', 2, 'linestyle', '--', 'color', c.roseRed )         
    
end
    

T = 6.3;                                                                    % Total Time of ode intergration
k1 = 3;  k2 = 4;                                                           % Position Error gain for xr, er, respectively.
a  = 0.2;  b = 2;                                                          % The coefficient (a) and exponential decay rate (b) of the sliding variable s

x0 = horzcat( qi, zeros(1,N) );                                            % Initial Condition of the simulation.
                                                                           % Setting the qe terms as 0

[ tvec,x ] = ode45( @(t,x) odefcn(t, x, k1, k2, a, b, xd, vd, robot.FK_func, robot.J_func ), [0, T], x0 );

% Since ode45 varies the step size, changing it to equivalent step size
tstep = 1e-4;
tmp = tstep * ( 1 : T/tstep ); 
tVec = tmp;

qMat = interp1( tvec, x( :, 1:N ), tmp ); 
tmp = arrayfun( xd, tVec , 'UniformOutput', false); tmp = cell2mat( tmp' );


%%
% Defining Tracking marker "tracker" for visualization
% Putting the time vector to desired x position for marker definition
tracker = myMarker( tmp( :, 1), tmp( :, 2), zeros( 1, length( tVec ) ),    ... % Z Position is zero since 
                    'name', "trackingMarker" , 'markersize', 7, 'markercolor', c.pink ); 

robot.addTrackingPoint( tracker )                
clear tmp*
robot.runAnimation( tVec, qMat', 0.33, false );


%% (--) ODE Function Definition.                   
function dx = odefcn( t, x, k1, k2, a, b, xd, vd, FK_func, J_func  )
    %  n is the number of DOF of the robot.
    %  x = 2n-by-1 vector, express q and qe in order, where each elements are n-by-1-vector
    % dx is the time difference of x, which will be used for the ode function.
    
    n = nargin( FK_func );                                                 % number of DOF of the robot.
    dx = zeros( 2 * n, 1 );

    % For putting the input q as the input of FK_func and J_func, changing the array to cell
    % [REF] https://www.mathworks.com/matlabcentral/answers/524984-using-an-array-as-input-argument-of-a-function-handle
    q = num2cell( x( 1 : n ) ); 
    
    pEE = FK_func( q{:} ); 
    J   =  J_func( q{:} );           

    if n == 2
        
        dx( 1 : n ) = inv( J ) * ( vd( t )  - k1 * ( pEE' - xd( t ) ) )' + a * exp( -b * t );
        dx( n+1 : 2*n ) =       dx( 1 : n ) + k2 * ( x( 1 : n) - x( n+1: 2*n) ) - a * exp( -b * t );
        
    else    % If redundent manipulator
        Jinv = J' * inv( J * J' );
        dx(   1 :     n ) = Jinv * ( vd( t )  - k1 * ( pEE' - xd( t ) ) )' + a * exp( -b * t ) - 3 * ( eye(n,n) - Jinv * J ) * x( 1 : n );
        dx( n+1 : 2 * n ) =       dx( 1 : n ) + k2 * ( x( 1 : n ) - x( n+1: 2*n ) ) - a * exp( -b * t );        
    end
        
   
end
