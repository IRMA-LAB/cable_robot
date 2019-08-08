function vect = IntegrableInverseDynamics33(cdpr_p,cdpr_v,...
        sim_data,index,time,orientation,period,k,geom_fun,record)
  
    pStart = sim_data.p(1:cdpr_p.n_cables,index);
    pEnd = sim_data.p(1:cdpr_p.n_cables,index+1);
    normalizedTime = NormalizedTime(k,time,period);
    dofs = ActuatedDofs(sim_data,normalizedTime,...
        geom_fun,pStart,pEnd);
    
    cdpr_v = UpdateIKZeroOrd(dofs(1,:)',orientation(1:3),cdpr_p,cdpr_v);
    cdpr_v = UpdateIKFirstOrd(dofs(2,:)',orientation(4:6),cdpr_p,cdpr_v);
    cdpr_v.platform.acceleration = dofs(3,:)';
    cdpr_v.platform = cdpr_v.platform.UpdateMassMatrixStateSpace(cdpr_p);
    cdpr_v = CalcTotalLoadsStateSpace(cdpr_v,cdpr_p);
    cdpr_v = UnderactuatedDynamicsPartition(cdpr_v);
    cdpr_v = UnderactuatedDynamicsAcceleration(cdpr_v);

    vect(1:3,1) = orientation(4:6);
    vect(4:6,1) = cdpr_v.platform.orientation_deriv_2;
    
    %record.SetFrame(cdpr_v,cdpr_p);

end