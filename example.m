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
              
c  = myColor();          

%% (1A) 2-Link Manipulator Example #1.
% Tracking a circular trajectory with constant angular acceleration.
clc

l1  =   2;    l2 = 3;  
q1i = 0.1; q2i = 0.2; 

robot = myPlanarRobot( [l1, l2] );
robot.initializeAnimation( [q1i, q2i] )

tmpLim = sum( [l1, l2] ) + 2;
set( robot.hAxes( 1 ),   'XLim',   [ -tmpLim , tmpLim ] , ...                  
                         'YLim',   [ -tmpLim , tmpLim ] )


% [PARAMETER SETTINGS]
% Setting the parameters for sliding-variable position tracking.
% The equation of desired trajectory's position + velocity w.r.t. time.
syms t
R  = 4;                                                                    % Radius of the Desired Trajectory
xd = [R * cos( 1/2 * t.^2 ), R * sin( 1/2 * t.^2 )];                       % Desired Trajectory Position w.r.t. time
vd = diff( xd, t );                                                        % Desired Trajectory Velocity w.r.t. time 
xd = matlabFunction( xd ); vd = matlabFunction( vd );                      % Converting symbolic expression as function handle.          

% Drawing the circle to track.                   
tmp = 0 : 0.001 : 2*pi;
plot( R * cos( tmp ), R * sin( tmp ), 'linewidth', 3, 'linestyle', '--', 'color', c.blue_sky )                    

T = 5;                                                                     % Total Time of ode intergration
k1 = 3;  k2 = 4;                                                           % Position Error gain for xr, er, respectively.
a  = 0.2;  b = 2;                                                          % The coefficient (a) and exponential decay rate (b) of the sliding variable s

q1i= 0.3; q2i = 0.3;                                                       % Initial Posture of the robot, shoulder and elbow joint angle respectively.
x0 = [q1i, q2i, 0, 0];                                                     % Initial Condition of the simulation.

[ tvec,x ] = ode45( @(t,x) odefcn(t, x, k1, k2, a, b, xd, vd, robot.FK_func, robot.J_func ), [0, T], x0 );


% Since ode45 varies the step size, changing it to equivalent step size
tstep = 1e-4;
tmp = tstep * ( 1 : T/tstep ); 
tVec = tmp;

q1 = interp1( tvec, x( :, 1 ), tmp ); q2 = interp1( tvec, x( :, 2 ), tmp );

clear tmp*
robot.runAnimation( tVec, [q1;q2], 1, false );

%% (1B) 3-Link Manipulator Example #1.
clc
l1 = 2; l2 = 3; l3 = 3;
robot = myPlanarRobot( [l1, l2, l3] );

q1i= 0.3; q2i = 0.3; q3i = 0.2;                                            % Initial Posture of the robot, shoulder and elbow joint angle respectively.
robot.initializeAnimation( [q1i, q2i, q3i] )

tmpLim = sum( [l1, l2, l3] ) + 2;
set( robot.hAxes( 1 ),   'XLim',   [ -tmpLim , tmpLim ] , ...                  
                         'YLim',   [ -tmpLim , tmpLim ] )

% [PARAMETER SETTINGS]
% Setting the parameters for sliding-variable position tracking.
% The equation of desired trajectory's position + velocity w.r.t. time.
syms t
R  = 4;                                                                    % Radius of the Desired Trajectory
xd = [R * cos( 1/2 * t.^2 ), R * sin( 1/2 * t.^2 )];                       % Desired Trajectory Position w.r.t. time
vd = diff( xd, t );                                                        % Desired Trajectory Velocity w.r.t. time
 
xd = matlabFunction( xd ); vd = matlabFunction( vd );                      % Converting symbolic expression as function handle.          
% Drawing the circle to track.                   
tmp = 0 : 0.001 : 2*pi;
plot( R * cos( tmp ), R * sin( tmp ), 'linewidth', 3, 'linestyle', '--', 'color', c.blue_sky )                    


T = 5;                                                                     % Total Time of ode intergration
k1 = 3;  k2 = 4;                                                           % Position Error gain for xr, er, respectively.
a  = 0.2;  b = 2;                                                          % The coefficient (a) and exponential decay rate (b) of the sliding variable s


x0 = [q1i, q2i, q3i, 0, 0, 0];                                             % Initial Condition of the simulation.
[ tvec,x ] = ode45( @(t,x) odefcn(t, x, k1, k2, a, b, xd, vd, robot.FK_func, robot.J_func), [0, T], x0 );

% Since ode45 varies the step size, changing it to equivalent step size
tstep = 1e-4;
tmp = tstep * ( 1 : T/tstep ); 
tVec = tmp;

q1 = interp1( tvec, x( :, 1 ), tmp ); 
q2 = interp1( tvec, x( :, 2 ), tmp );
q3 = interp1( tvec, x( :, 3 ), tmp );

clear tmp*
robot.runAnimation( tVec, [q1;q2;q3], 1, false );

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
        dx(   1 :     n ) = Jinv * ( vd( t )  - k1 * ( pEE' - xd( t ) ) )' + a * exp( -b * t ) + ( eye(n,n) - Jinv * J ) * x( 1 : n );
        dx( n+1 : 2 * n ) =       dx( 1 : n ) + k2 * ( x( 1 : n ) - x( n+1: 2*n ) ) - a * exp( -b * t );        
    end
        
   
end
