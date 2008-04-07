function q = ikine_planar_pa10(T_w_t, rs)

[q,converged] = inv_kin(rs, T_w_t);


% T_0_3 = inv_tmatrix(rs.T_0) * T_w_t * inv_tmatrix(rs.T_t);
% 
% x = T_0_3(1,4);
% y = T_0_3(2,4);
% c123 = T_0_3(1,1);
% s123 = T_0_3(2,1);
% 
% % Map p -> q
% % q2
% c2 = (x^2 + y^2 - rs.l1^2 - rs.l2^2) / (2*rs.l1*rs.l2);
% if ( abs(c2) > 1 )
%     q = [];
%     return;
% end
% s2 = sqrt(1 - c2^2);
% q2 = [atan2(s2, c2); atan2(-s2, c2)];
% 
% % q1
% c2 = cos(q2); s2 = sin(q2);
% k1 = rs.l1 + rs.l2*c2;
% k2 = rs.l2*s2;
% q1 = atan2(y,x) - atan2(k2,k1);
% 
% % q3
% phi = atan2(s123, c123);
% q3 = phi - q1 - q2;
% 
% q = [q1, q2, q3];
% 
% % Check solution against joint limits
% % Check each solution to see if it is outside its joint limits
% qmin = rs.qmin;
% qmax = rs.qmax;
% good_soln = [1; 1];
% 
% for ii = 1:2
%     for jj = 1:3
%                 
%         % Check angle against upper range of joint
%         if ( q(ii,jj) > qmax(jj) )
%             % Flip angle around unit circle. Check if this new angle
%             % exceeds the lower range of the joint.
%             q(ii,jj) = q(ii,jj) - 2*pi;
%             
%             if ( q(ii,jj) < qmin(jj) )
%                 good_soln(ii) = 0;
%                 break;
%             end            
%         end
%         
%         % Check angle against lower range of joint
%         if ( q(ii,jj) < qmin(jj) )
%             % Flip angle around unit circle. Check if this new angle
%             % exceeds the upper range of the joint.
%             q(ii,jj) = 2*pi + q(ii,jj);
%             
%             if ( q(ii,jj) > qmax(jj) )
%                 good_soln(ii) = 0;
%                 break;
%             end            
%         end
%     end
% end
% 
% ind = find(good_soln == 1);
% q = q(ind,:);