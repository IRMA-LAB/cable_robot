 function stab = AssertStability33BruteForce(cdpr_p,pose,pos_g,l,rec,ut)
 i=0;
 for alpha = 0:pi/180:2*pi
      i = i+1;
      [pose_new,fval,extiflag,output] = fsolve(@(v)CalcDirectKinematicsLengthStab(cdpr_p,rec,[l;alpha],v),pose,ut.fsolve_options);
      cdpr_v = CdprVar(cdpr_p.n_cables);
      cdpr_v = UpdateIKZeroOrd(pose_new(1:3,1),pose_new(4:end,1),cdpr_p,cdpr_v);
      delta = cdpr_v.platform.pos_OG_glob(3)-pos_g(3);
      if (delta>0)
          stabb(i,1) = 0;
      else
          stabb(i,1) = 1;
      end
      
 end
      stab = any(stabb);
 end