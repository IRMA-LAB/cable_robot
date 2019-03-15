function cdpr_v = UpdateIKZeroOrd(position,orientation,cdpr_p,cdpr_v)

cdpr_v.platform = UpdatePlatformPose(position,orientation,...
  cdpr_p.rotation_parametrization,cdpr_p.platform.pos_G_loc,cdpr_v.platform);
for i=1:length(cdpr_v.cable)
  cdpr_v.cable(i) = UpdateCableZeroOrd(cdpr_p.cable(i),cdpr_v.platform,cdpr_v.cable(i));

end