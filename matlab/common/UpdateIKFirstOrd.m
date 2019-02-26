function cdpr_v = UpdateIKFirstOrd(velocity,orientation_d,cdpr_p,cdpr_v)

cdpr_v.platform = UpdatePlatformVelocity(velocity,orientation_d,...
  cdpr_p.rotation_parametrization,cdpr_v.platform);
for i=1:length(cdpr_v.cable)
  cdpr_v.cable(i) = UpdateCableFirstOrd(cdpr_p.cable(i),cdpr_v.platform,cdpr_v.cable(i));
  cdpr_v.geometric_jacobian(i,:) = cdpr_v.cable(i).geometric_jacobian_row;
  cdpr_v.analitic_jacobian(i,:) = cdpr_v.cable(i).analitic_jacobian_row;
end

end