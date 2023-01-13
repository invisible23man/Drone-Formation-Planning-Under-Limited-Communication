function ci = get_base_config(ci_1, ci_2, z, dim)
%GET_BASE_CONFIG get the base configuration 
% for the robots as a unit grid configuration
%

if ~exist('ci_1', 'var')
    ci_1 = [-1 0 1 ]; 
end

if ~exist('ci_2', 'var')
    ci_2 = [-1 0 1 ];
end

[X,Y] = meshgrid(ci_1, ci_2);

ci = [X(:) Y(:)];

if exist('dim', 'var') 
    % create 3d base config
    if ~exist('z', 'var')
        z = 1;
    end

    ci = [X(:) Y(:) z*ones(size(ci,1),1)];
end

end

