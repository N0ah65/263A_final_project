classdef NumGenerator
    properties
        vertices
        digit_segments
    end
    
    methods
        function obj = NumGenerator()
            % Coordinates for each vertex of a 7-segment display
            obj.vertices = struct( ...
                'A', [-0.05+0.1, 0.32, 0], ...  % Top left
                'B', [0+0.1, 0.32, 0], ...      % Top right
                'C', [-0.05+0.1, 0.22, 0], ...  % Middle left
                'D', [0+0.1, 0.22, 0], ...      % Middle right
                'E', [-0.05+0.1, 0.12, 0], ...  % Bottom left
                'F', [0+0.1, 0.12, 0] ...       % Bottom right
            );

           
         
            % The pattern for the 7-segment display digits
            obj.digit_segments = containers.Map( ...
                {0, 1, 2, 3, 4, 5, 6, 7, 8, 9}, ...
                { ...
                    {'A', 'B', 'D', 'F', 'E', 'C', 'A'}, ...
                    {'B', 'D', 'F'}, ...
                    {'A', 'B', 'D', 'C', 'E', 'F'}, ...
                    {'A', 'B', 'D', 'C', 'D', 'F', 'E'}, ...
                    {'A', 'C', 'D', 'B', 'D', 'F'}, ...
                    {'B', 'A', 'C', 'D', 'F', 'E'}, ...
                    {'B', 'A', 'C', 'E', 'F', 'D', 'C'}, ...
                    {'A', 'B', 'D', 'F'}, ...
                    {'C', 'A', 'B', 'D', 'C', 'E', 'F', 'D'}, ...
                    {'D', 'C', 'A', 'B', 'D', 'F', 'E'} ...
                });
        end
        
        function points = interpolate_points(~, start, stop, num_points)
            x_vals = linspace(start(1), stop(1), num_points);
            y_vals = linspace(start(2), stop(2), num_points);
            z_vals = linspace(start(3), stop(3), num_points); % 支持 Z 轴
            points = [x_vals', y_vals', z_vals'];
        end
        
        function pos_array = generate_coord(obj, number, points, scaling, deltaxyz)
            if nargin < 4, scaling = 1; end
            if nargin < 5, deltaxyz = [0, 0, 0]; end
            
            % Get active vertices for the number
            active_v = obj.digit_segments(number);
            pos_array = [];
            
            for i = 1:(length(active_v) - 1)
                start = obj.vertices.(active_v{i});
                stop = obj.vertices.(active_v{i+1});
                interpolated_points = obj.interpolate_points(start, stop, points);
                
                % Apply scaling and offset
                interpolated_points = interpolated_points * scaling + deltaxyz;
                pos_array = [pos_array; interpolated_points]; %#ok<AGROW>
            end
        end
    end
end
