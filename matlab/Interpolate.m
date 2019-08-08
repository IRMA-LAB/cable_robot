function [pos,vel,acc] = Interpolate(sp,time,dt)

pos = ppval(sp,time);

% if (time>=4*dt)
%    pm1 = ppval(sp,time-dt);
%    pm2 = ppval(sp,time-2.*dt);
%    pm3 = ppval(sp,time-3.*dt);
%    pm4 = ppval(sp,time-4.*dt);
%    M = [1 0      0        0        0;
%         1 (dt)   (dt)^2   (dt)^3   (dt)^4; 
%         1 (2*dt) (2*dt)^2 (2*dt)^3 (2*dt)^4;
%         1 (3*dt) (3*dt)^2 (3*dt)^3 (3*dt)^4;
%         1 (4*dt) (4*dt)^2 (4*dt)^3 (4*dt)^4];
%    F = [pm4;pm3;pm2;pm1;pos];
%    c = linsolve(M,F);
%    vel = [0 1 2*(4*dt) 3*(4*dt)^2 4*(4*dt)^3]*c;
%    acc = [0 0 2 6*(4*dt) 12*(4*dt)^2]*c;
% else 
%    if (time>=3*dt)
%       pm1 = ppval(sp,time-dt);
%       pm2 = ppval(sp,time-2.*dt);
%       pm3 = ppval(sp,time-3.*dt);
%       M = [1 0      0        0;
%            1 (dt)   (dt)^2   (dt)^3; 
%            1 (2*dt) (2*dt)^2 (2*dt)^3;
%            1 (3*dt) (3*dt)^2 (3*dt)^3];
%       F = [pm3;pm2;pm1;pos];
%       c = linsolve(M,F);
%       vel = [0 1 2*(3*dt) 3*(3*dt)^2]*c;
%       acc = [0 0 2 6*(3*dt)]*c;
%    else
     if (time>=2*dt)
      pm1 = ppval(sp,time-dt);
      pm2 = ppval(sp,time-2.*dt);
      M = [1 0      0
           1 (dt)   (dt)^2; 
           1 (2*dt) (2*dt)^2];
      F = [pm2;pm1;pos];
      c = linsolve(M,F);
      vel = [0 1 2*(2*dt)]*c;
      acc = [0 0 2]*c;
    else if (time>=dt)
        pm1 = ppval(sp,time-dt);
        vel = (pos-pm1)/dt;
        acc = 0;
      else 
        vel = 0;
        acc = 0;
      end
      
    end
    
%   end
%   
% end
    

end