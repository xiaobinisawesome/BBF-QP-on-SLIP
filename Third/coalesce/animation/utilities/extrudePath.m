function [x, y, z] = extrudePath(path, radius)
%EXTRUDEPATH Extrudes a cylinder from a 3D path.
%
% Syntax:
%   [x, y, z] = extrudePath(path, radius) extrudes a cylinder
%   along the curved path = [x, y, z] with a given radius.
%
% Description:
%   Much like the built in CYLINDER function but works with curved 3D paths
%   as well.
%
% Copyright 2013-2014 Mikhail S. Jones

	% Set options
	threshold = 0.5*radius;
	nRadiusPoints = 15;

	% Collapse points within THRESHOLD of each other
	nPathPoints = 1;
	for k = 2:(size(path, 2) - 1)
		if (norm(path(:,k) - path(:,nPathPoints)) > threshold)
			nPathPoints = nPathPoints + 1;
			path(:,nPathPoints) = path(:, k);
		end % if
	end % for

	% Always include endpoint
	if (norm(path(:,end) - path(:,nPathPoints)) > 0)
		nPathPoints = nPathPoints + 1;
		path(:,nPathPoints) = path(:, end);
	end % if

	% Average for internal points, first stretch for endpoints
	dv = path(:, [2:end, end]) - path(:, [1, 1:end-1]);

	% Make nvec not parallel to dv(:,1)
	nvec = zeros(3, 1);
	[~, idx] = min(abs(dv(:,1)));
	nvec(idx) = 1;
	xyz = zeros([3, nRadiusPoints + 1, nPathPoints + 2]);

	% Compute cos and sin
	c = repmat(cos(linspace(0, 2*pi, nRadiusPoints + 1)), [3, 1]);
	s = repmat(sin(linspace(0, 2*pi, nRadiusPoints + 1)), [3, 1]);

	% Propagate the normal (nvec) along the tube
	for k = 1:nPathPoints
		convec = cross(nvec, dv(:,k));
		convec = convec./norm(convec);
		nvec = cross(dv(:,k), convec);
		nvec = nvec./norm(nvec);
		xyz(:,:,k+1) = repmat(path(:,k), [1, nRadiusPoints + 1]) + ...
			c.*repmat(radius*nvec, [1, nRadiusPoints + 1]) + ...
			s.*repmat(radius*convec, [1, nRadiusPoints + 1]);
	end % for

	% Cap the ends
	xyz(:,:,1) = repmat(path(:,1), [1, nRadiusPoints + 1]);
	xyz(:,:,end) = repmat(path(:,end), [1, nRadiusPoints + 1]);

	% Extract results
	x = squeeze(xyz(1,:,:));
	y = squeeze(xyz(2,:,:));
	z = squeeze(xyz(3,:,:));
end % extrudePath
