function [geometric,analitic] = CalcPlatformJacobianRowDerivatives(vers_rho,vers_rho_d,pos_PA_glob,H_mat,H_mat_d,omega)

  geometric = [vers_rho_d' -(vers_rho_d'+vers_rho'*Anti(omega))*Anti(pos_PA_glob)];
  analitic = geometric;
  analitic(1,4:end) = analitic(1,4:end)*H_mat-vers_rho'*Anti(pos_PA_glob)*H_mat_d;
  
  
end