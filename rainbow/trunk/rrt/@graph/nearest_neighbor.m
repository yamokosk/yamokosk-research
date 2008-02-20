function [x_near, x_near_ind, u,dq_mag,ang] = nearest_neighbor(T,x)

v = T.v;
e = T.e;

v = v(1:T.v_ptr,:);

nr = size(v,1);

wq = 1;

% Separate the state data so its easier to manipulate
vq = v(:,1:6);  
% vqp = v(:,7:12);
q = x(1:6,1)';
% qp = x(7:12,1)';

% First compute the distance vectors from the test state to all other 
% states
qm = repmat(q,nr,1);
dq = vq - qm; % distance vectors
dq_mag = sqrt( diag(dq * dq') );
dq_mag_mat = repmat(dq_mag, 1, 6);
dq_norm = dq ./ dq_mag_mat;

% Next we need to compute the angle between the velocity vectors of all
% states in the tree to the new state
% vqp_mag = sqrt( diag(vqp * vqp') );
% 
% vqp_mag_zeros = find( vqp_mag == 0 );
% vqp_mag(vqp_mag_zeros,:) = 1e-6;
% 
% dq_mag_mat = repmat(vqp_mag, 1, 6);
% vqp_norm = vqp ./ dq_mag_mat;
% 
% cos_ang = dot(dq_norm,vqp_norm,2);
% ang = acos(cos_ang);
% 
% Create a weighted sum of the two measures
% dt = wq*dq_mag + (1-wq)*ang;
dt = dq_mag;

% Finally, pick out the index of the state with the minimum distance.
x_near_ind = find( dt == min(dt) );

x_near = v(x_near_ind,:)';

% Lastly.. pick out the control that brought us to x_near
edge_ind = find(e(:,2) == x_near_ind);

u = zeros(6,1);
if (~isempty(edge_ind))
    u = e(edge_ind,4:9)';
end
%     params = pa10_params();
%     u = pa10_invdyn(x_near(1:6,1),zeros(6,1),zeros(6,1),params);
%end