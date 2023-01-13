function eta_Nis = get_neigbour_etas(cf_num, eta)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% Unit Grid Configuration
% [-1,0,1;    o3 o6 o9
%  -1,1,1;    o2 o5 o8
%  -1,2,1;    o1 o4 o7
%   0,0,1;
%   0,1,1;
%   0,2,1;
%   1,0,1;
%   1,1,1;
%   1,2,1
% ]

neighbour_map_ugrid = [[0,2,0,4,5,0,0,0,0]
                      [1,0,3,4,5,6,0,0,0]
                      [0,2,0,0,5,6,0,0,0]
                      [1,2,0,0,5,0,7,8,0]
                      [1,2,3,4,0,6,7,8,9]
                      [0,2,3,0,5,0,0,8,9]
                      [0,0,0,4,5,0,0,8,0]
                      [0,0,0,4,5,6,7,0,9]
                      [0,0,0,0,5,6,0,8,0]
                    ];

% get neighbour drones                
neighbour_cfs = neighbour_map_ugrid(cf_num,:); 
eta_Nis = zeros(nnz(neighbour_cfs),size(eta,2));
% eta_Nis = eta;
% eta_Nis(cf_num,:) = [];
nghbr = 1;
for j=1:length(neighbour_map_ugrid)
    if neighbour_cfs(j)
        eta_Nis(nghbr,:) = eta(j,:);
        nghbr = nghbr+1; 
    end
end

end

