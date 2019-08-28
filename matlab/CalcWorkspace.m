function out = CalcWorkspace(cdpr_p,cdpr_v,out,ut,type,set_point)



if (type == 1) % translational
  
  cfr_arc_min = 0.02;
  z_step = 0.02;
  r_step = 0.02;
  counter = 0;
  if (cdpr_p.n_cables <6) % underactuated
  gStat_mask = [1;1;1;0;0;0];
  start_idx = 1
  hold on
  for z = 0.1:-z_step:-1.5
    in_guess = [0;0;0];
    for r = 0:r_step:1.5
      cfr = 2*pi*r;
      steps = ceil(cfr./cfr_arc_min);
      for ang =0:2*pi/steps:2*pi*(1-1./steps)
        
        p = cdpr_p.workspace_center+[r*cos(ang);r*sin(ang);z];
        %tic
        or = fsolve(@(v) CalcWPGeometricStatic(cdpr_p, p, v, gStat_mask),...
        in_guess, ut.fsolve_options_grad);
        %toc
        in_guess = or;
        pose = [p;or];
        cdpr_v = UpdateIKZeroOrd(pose(1:3,1),pose(4:end,1),cdpr_p,cdpr_v);
        cdpr_v = CalcExternalLoadsStateSpace(cdpr_v,cdpr_p,eye(3));
        cdpr_v = CalcCablesTensionStat(cdpr_v);
        check1 = ~any(isnan(cdpr_v.tension_vector./cdpr_p.platform.mass)); 
        if (check1)
          check2 = ~(any(cdpr_v.tension_vector./cdpr_p.platform.mass < 2) || any(cdpr_v.tension_vector./cdpr_p.platform.mass > 10));
          if (check2) 
            counter = counter + 1;
            out.pose_WP(:,counter) = pose;
            out.condJ(1,counter) = cond(cdpr_v.geometric_jacobian);
            out.netZ_tension_vector_WP(:,counter) = cdpr_v.tension_vector./cdpr_p.platform.mass;
          end
        end
      end
    end
    tMax = max(out.netZ_tension_vector_WP(:,start_idx:counter));
    [ss,s_i] = sort(tMax);
    xx = out.pose_WP(1,start_idx:counter);
    yy = out.pose_WP(2,start_idx:counter);
    zz = out.pose_WP(3,start_idx:counter);
    cmap = jet(length(xx));
    scatter3(xx(s_i),yy(s_i),zz(s_i),10,cmap,'filled')
    start_idx = counter;
  end
  hold off
  
    
  elseif (cdpr_p.n_cables >6) % overactuated
    
  else % completely actuated
  
  end
  
else  % orientational
  
  if (cdpr_p.n_cables <6) % underactuated
  
  elseif (cdpr_p.n_cables >6) % overactuated
    
  else % completely actuated
  
  end
  
end

tMax = max(out.netZ_tension_vector_WP);
[ss,s_i] = sort(tMax);
xx = out.pose_WP(1,s_i);
yy = out.pose_WP(2,s_i);
zz = out.pose_WP(3,s_i);
cmap = jet(length(xx));
scatter3(xx,yy,zz,10,cmap,'filled')
end