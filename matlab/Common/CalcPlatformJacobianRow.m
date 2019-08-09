function [geometric, analitic] = CalcPlatformJacobianRow(vers_rho, pos_PA_glob, H_mat)

geometric = [vers_rho' (-vers_rho' * Anti(pos_PA_glob))];
analitic = geometric;
analitic(1, 4:end) = analitic(1, 4:end) * H_mat;

end