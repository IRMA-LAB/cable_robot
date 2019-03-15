classdef CableVar
  properties
    length;
    swivel_ang;
    tan_ang;
    
    pos_PA_glob;
    pos_OA_glob;
    pos_DA_glob;
    pos_BA_glob;
    
    vers_u;
    vers_w;
    vers_n;
    vers_rho;
    
    speed;
    swivel_ang_vel;
    tan_ang_vel;
    
    vel_OA_glob;
    vel_BA_glob;
    
    vers_u_deriv;
    vers_w_deriv;
    vers_n_deriv;
    vers_rho_deriv;
    
    acceleration;
    swivel_ang_acc;
    tan_ang_acc;
    
    acc_OA_glob;
    
    geometric_jacobian_row;
    analitic_jacobian_row;
  end
  methods
    function obj = Update(obj,l,s,t)
      obj.length = l;
      obj.swivel_ang = s;
      obj.tan_ang = t;
    end
  end
end