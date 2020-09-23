classdef myPlanarRobot < handle
% % =============================================================== %
%   [DESCRIPTION]
%       Class for generating an n-link planar robot 
%
%   [CREATED BY]  Moses Nah
%   [EMAIL]       mosesnah@mit.edu
%   [AFFILIATION] Massachusetts Institute of Technology (MIT)
%   [MADE AT]     19-September-2020    
% % =============================================================== %    
    properties ( SetAccess = private )
        
                                           % [Configuration #1] A single big plot
        pos1m  = [0.08 0.10 0.84 0.80];    %   Position/size for the main plot before adding plot 
        
                                           % [Configuration #2] A big plot on the left, two plots on the upper/lower right.
        pos2   = [0.08 0.08 0.42 0.82; ... %   Position/size for the main plot - 3D real time plot
                  0.58 0.08 0.40 0.37; ... %   Position/size for the under-sub-plot, drawn in the lower section
                  0.58 0.55 0.40 0.40]     %   Position/size for the above-sub-plot, drawn in the upper section.       
    end
        
    properties ( SetAccess = public )
        
        lengths;                                                           % Length of each limb.
              N;                                                           % Number of links (limbs) of the planar robot.
         J_sym;  J_func;                                                   % End-effector Jacobian, symbolic and function.     
        FK_sym; FK_func;                                                   % End-effector Forward-Kinematics, symbolic and function.
        
        hFig    = gobjects( 0 );                                           % Handle of figure.
        hTitle  = gobjects( 0 );                                           % Handle of Title. 
        hAxes   = gobjects( 1, 3 );                                        % Handles of axes, The list of axes, 1,2,3 is ordered as main, side1, side2
        
        % 1, 2 and 3 correspond to main plot, subplot1 and subplot2, respectively.
        hMarkers  = {     gobjects(0),     gobjects(0),       gobjects(0) };
        markers   = {  myMarker.empty,  myMarker.empty,    myMarker.empty };      
        
        hLimbs; hJoints

        c = myColor(); 
        cMat;
    end
    
    methods (Static)
                
        function p = polar2xy( r, phi )
             p = [ r * cos( phi ), r * sin( phi ) ]; 
        end 
    end
        
    
    methods
        function obj = myPlanarRobot( l )
            %Constructor of the robot-link
            obj.lengths = l;                                               % Saving the lengths of 
            obj.N       = length( l );                                     % Number of links needed.
            
            % Defining the symbolic array for calculating the forward kinematics + jacobian.   
            qVec_sym = sym( 'q',[1, obj.N], 'real' );

            pMat = obj.forwardKinematics( qVec_sym );                      % Calculating the whole 
            
            r1 = gradient( pMat( 1, end ), qVec_sym )';
            r2 = gradient( pMat( 2, end ), qVec_sym )';            
            
            obj.FK_sym  = pMat( :, end );
            obj.FK_func = matlabFunction( obj.FK_sym );
            
            obj.J_sym  = [r1; r2];                                         % Symbolic   expression of the Jacobian
            obj.J_func = matlabFunction( obj.J_sym );                      % Functional expression of the Jacobian        
            
            % The graphic objects for the link lines and joint markers
            obj.hLimbs  = gobjects( 1, obj.N     );
            obj.hJoints = gobjects( 1, obj.N + 1 );
            
            obj.cMat    = [obj.c.green;   obj.c.blue;   obj.c.yellow; ...
                            obj.c.pink; obj.c.orange; obj.c.blue_sky; 
                          obj.c.purple;   obj.c.grey; obj.c.orange_milky; obj.c.roseRed];
            
        end
        
        function J = calculateJacobian( obj, qVec )
            
            J = obj.J_func( qVec );
            
        end        

        function pMat = forwardKinematics( obj, qVec )

            v1 = tril( repmat( obj.lengths, obj.N, 1 ) ) * cos( cumsum( qVec ) )'; 
            v2 = tril( repmat( obj.lengths, obj.N, 1 ) ) * sin( cumsum( qVec ) )';

            pMat = [v1'; v2'];
            pMat = [ zeros(2,1), pMat ];                                   % Add zeros(2,1) for origin.
        end
        
        
        function initializeAnimation( obj, qVec_init )
            % Initialize the animation for the problem.
            
            % Setting the default figure and Axes for the plot
            obj.hFig        = figure();
            obj.hAxes( 1 )  = subplot( 'Position', obj.pos1m, 'parent', obj.hFig );    
            hold( obj.hAxes( 1 ),'on' ); axis( obj.hAxes( 1 ) , 'equal' )             

            pMat = obj.forwardKinematics( qVec_init );                     % 2-by-N+1 matrix, each column is the position of each joints
            
            for i = 1 : obj.N
                obj.hLimbs( i ) = plot( [ pMat( 1, i ), pMat( 1, i + 1 ) ], ...
                                        [ pMat( 2, i ), pMat( 2, i + 1 ) ], ...
                                        'linewidth', 4, ...
                                        'color', obj.c.grey );
            end
            
            for i = 1 : obj.N + 1
                obj.hJoints( i ) = plot(  pMat( 1, i ), pMat( 2, i ), 'o', ...
                                          'MarkerEdgeColor', obj.cMat( i, : ), ...
                                          'MarkerSize'     , 10, ...
                                          'MarkerFaceColor', [1,1,1] );  
            end
                    
        end
        
        function addTrackingPoint( obj, marker )
              m = plot( marker.xdata( 1 ), marker.ydata( 1 ), 'o', ...
                                             'parent', obj.hAxes( 1 ), ...
                                             'Marker', marker.markerStyle, ...
                                         'MarkerSize', marker.markerSize,  ...
                                    'MarkerEdgeColor', marker.markerColor, ...
                                    'MarkerFaceColor',  [1,1,1]  ); 

             obj.hMarkers{ 1 }( end + 1 ) = m;
             obj.markers{ 1 }( end + 1 )  = marker; 
        end
        
        function runAnimation( obj, tVec, qVec, vidSpeed, isRecord )
            
            obj.hTitle  = title( obj.hAxes( 1 ), sprintf( '[Time] %5.3f (s)', tVec( 1 ) ) );
            
            Nt   = length( tVec );
            dt   = tVec( 2 ) - tVec( 1 );
            step = round( ( 1 / dt / 60 ) );                               % Setting 60 fps - 1 second as default!     
            
            if step == 0                                                   % In case the step is too small, then set the simStep as 1
               step = 1; 
            end
            
            if isRecord                                                    % If video record is ON

                fps     = 60 * vidSpeed;                                   % Frame-per-second of the video, 60Hz is default rate.
                writerObj = VideoWriter( [num2str( obj.N ),'link_ani'], 'MPEG-4' );          % Saving the video as "videoName" 
                writerObj.FrameRate = fps;                                 % How many frames per second.
                open( writerObj );                                         % Opening the video write file.

            end    
            
            for i = 1 : step : Nt
                
                set( obj.hTitle, 'String', sprintf( '[Time] %5.3f (s)  x%2.1f', tVec( i ), vidSpeed ) );            
                
                % Calculating the forward kinematics of the system.
                pMat = obj.forwardKinematics( qVec( :, i )' );
                
                for j = 1 : obj.N
                    set( obj.hLimbs(  j    ), 'xdata', [ pMat( 1,j ), pMat( 1,j+1 ) ], 'ydata', [ pMat( 2,j ), pMat( 2,j+1 ) ] );
                    set( obj.hJoints( j + 1), 'xdata', pMat( 1, j + 1 ), 'ydata', pMat( 2, j + 1 ) );
                end
                
                % Update Tracking Point
                for j = 1 : length( obj.hMarkers )

                    if isempty( obj.hMarkers{ j } )
                        continue     % Go to next iteration
                    end

                    for k = 1 : length( obj.hMarkers{ j } )

                        set( obj.hMarkers{ j }( k ), 'XData', obj.markers{ j }( k ).xdata( i ), ...
                                                     'YData', obj.markers{ j }( k ).ydata( i )  )

                    end

                end                
                
                
                if isRecord                                                % If videoRecord is ON
                    frame = getframe( obj.hFig );                          % Get the current frame of the figure
                    writeVideo( writerObj, frame );                        % Writing it to the mp4 file/
                else                                                       % If videoRecord is OFF
                    drawnow                                                % Just simply show at Figure
                end

            end   

            if isRecord
                close( writerObj );
            end                     
            
            
            
        end

    end
end

