function color = myColor( )
% Returning the color structure for plotting.
%
% =============================================================== %
% [CREATED BY]: Moses C. Nah
%
% [DATE]: 07-June-2020
% =============================================================== %

% =============================================================== %
% [DESCRIPTION]
%   Function outputting a list of colors for plot
%
% =============================================================== %

% Color can also be expressed
color = struct(                                             ...
                'yellow'       , [0.9290, 0.6940, 0.1250] , ...
                'orange'       , [0.8500, 0.3250, 0.0980] , ...
                'pink'         , [0.9961, 0.4980, 0.6118] , ...
                'blue'         , [     0, 0.4470, 0.7410] , ...
                'purple'       , [0.4940, 0.1840, 0.5560] , ...
                'green'        , [0.4660, 0.6740, 0.1880] , ...
                'white'        , [   1.0,    1.0,    1.0] , ...
                'grey'         , [0.8200, 0.8200, 0.8200] , ...
                'roseRed'      , [0.7608, 0.1176, 0.3373] , ...
                'blue_sky'     , [0.4941, 0.7412, 0.7059] , ...
                'peach'        , [0.9647, 0.8196, 0.5961] , ...
                'purple_plum'  , [0.5255, 0.1647, 0.3608] , ...
                'orange_milky' , [0.9569, 0.6471, 0.2824] , ...
                'purple_dark'  , [0.1725, 0.0000, 0.2431]   ...
               );

end
