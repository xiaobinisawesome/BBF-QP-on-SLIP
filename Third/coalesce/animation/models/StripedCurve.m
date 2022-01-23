classdef StripedCurve < Model
%STRIPEDLINE Creates a diagonally striped line model object.

	properties
		width@double scalar = 1
		spacing@double scalar = 0.025
	end % properties

	methods
		function this = StripedCurve(width, spacing, xterrain, zterrain)

			% Call superclass constructor
			this = this@Model;

			% Parse input arguments and set default properties
% 			switch nargin
% 			case 0
% 				% Do nothing
% 			case 1
% 				this.width= width;
% 			case 4
				this.width= width;
				this.spacing = spacing;
% 			otherwise
% 				error('Invalid number of input arguments.');
% 			end % switch

			% Equations for a striped line
			xGrid = 0:spacing:width; yGrid = 0*xGrid;
			this.x = reshape([xGrid; xGrid - spacing; nan*xGrid], 1, []);
			this.y = reshape([yGrid; yGrid - spacing; nan*yGrid], 1, []);
			this.x = [this.x 0 width];
			this.y = [this.y+1 0 0];
			this.z = 0*this.x;

            this.x = xterrain; 
            this.y = zterrain; 
            this.z = 0*this.x; 
			% Create graphics object
			this.handle = plot3(this.x, this.y, this.z, ...
				'Color', 'black', ...
				'LineStyle', '-', ...
				'LineWidth', 2);
		end % StripedLine
        
        function thisCopy = copy(this, varargin)
            thisCopy = StripedCurve;
            
            thisCopy.width = this.width;
            thisCopy.spacing = this.spacing;
            thisCopy.x = this.x;
            thisCopy.y = this.y;
            thisCopy.z = this.z;
            set(thisCopy.handle, 'EdgeColor', this.color);
            set(thisCopy.handle, 'LineStyle', this.lineStyle);
            if ~isempty(varargin)
                alpha = varargin{1};
                set(thisCopy.handle, 'EdgeAlpha', alpha)
                set(thisCopy.handle, 'FaceAlpha', alpha)
                
            end
            %             thisCopy.handle = patch(this.x, this.y, this.z, this.color, ...
            %                 'EdgeColor', 'k', ...
            %                 'LineWidth', 2, ...
            %                 'LineStyle', this.lineStyle);
        end
	end % methods
end % classdef
