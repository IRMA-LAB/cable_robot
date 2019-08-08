function data = GetDestinations(data,cdpr_p,record,ut)

    data.pNumber = 13;
    data.p(1:3,1) = cdpr_p.workspace_center+[-0.3;0;0.0];
    reference_position = data.p(1:3,1);
     geometric_static_mask = [1;1;1;0;0;0];
    data.p(4:6,1) = fsolve(@(v) CalcGenericGeometricStatic(...
    cdpr_p,record,reference_position,v,geometric_static_mask),...
    [0;0;0],ut.fsolve_options);
  
    data.p(1:3,2)  = cdpr_p.workspace_center-[0.3;0;0.9];
    data.p(4:6,2)  = fsolve(@(v) CalcGenericGeometricStatic(...
    cdpr_p,record,data.p(1:3,2),v,geometric_static_mask),...
    [0;0;0],ut.fsolve_options);
    
    data.dt(1) = 1.5; 
    
    data.p(1:3,3)  = cdpr_p.workspace_center+[0.1;0;0];
    data.p(4:6,3)  = fsolve(@(v) CalcGenericGeometricStatic(...
    cdpr_p,record,data.p(1:3,3),v,geometric_static_mask),...
    [0;0;0],ut.fsolve_options);
    
    data.dt(2) = 1.5; 
    
    data.p(1:3,4)  = cdpr_p.workspace_center+[0.1;0;-0.9];
    data.p(4:6,4)  = fsolve(@(v) CalcGenericGeometricStatic(...
    cdpr_p,record,data.p(1:3,4),v,geometric_static_mask),...
    [0;0;0],ut.fsolve_options);
    
    data.dt(3) = 1.5; 
    
    data.p(1:3,5)  = cdpr_p.workspace_center+[0.5;0;0];
    data.p(4:6,5)  = fsolve(@(v) CalcGenericGeometricStatic(...
    cdpr_p,record,data.p(1:3,5),v,geometric_static_mask),...
    [0;0;0],ut.fsolve_options);
    
    data.dt(4) = 1.5; 
    
    data.p(1:3,6)  = cdpr_p.workspace_center+[0.5;0;-0.9];
    data.p(4:6,6)  = fsolve(@(v) CalcGenericGeometricStatic(...
    cdpr_p,record,data.p(1:3,6),v,geometric_static_mask),...
    [0;0;0],ut.fsolve_options);
    
    data.dt(5) = 2; 
    
    data.p(1:3,7)  = cdpr_p.workspace_center+[0.3;0.2;0];
    data.p(4:6,7)  = fsolve(@(v) CalcGenericGeometricStatic(...
    cdpr_p,record,data.p(1:3,7),v,geometric_static_mask),...
    [0;0;0],ut.fsolve_options);
    
    data.dt(6) = 1.5; 
    
    data.p(1:3,8)  = cdpr_p.workspace_center+[0.3;-0.2;-0.9];
    data.p(4:6,8)  = fsolve(@(v) CalcGenericGeometricStatic(...
    cdpr_p,record,data.p(1:3,8),v,geometric_static_mask),...
    [0;0;0],ut.fsolve_options);
    
    data.dt(7) = 1.5; 
    
    data.p(1:3,9) = cdpr_p.workspace_center+[-0.3;0;0.0];
    data.p(4:6,9) = fsolve(@(v) CalcGenericGeometricStatic(...
    cdpr_p,record,data.p(1:3,9),v,geometric_static_mask),...
    [0;0;0],ut.fsolve_options);

    data.dt(8) = 1.5; 

    data.p(1:3,10)  = cdpr_p.workspace_center+[0.3;0.2;-0.9];
    data.p(4:6,10)  = fsolve(@(v) CalcGenericGeometricStatic(...
    cdpr_p,record,data.p(1:3,10),v,geometric_static_mask),...
    [0;0;0],ut.fsolve_options);

    data.dt(9) = 1.5; 
    
    data.p(1:3,11)  = cdpr_p.workspace_center+[0.3;-0.2;0];
    data.p(4:6,11)  = fsolve(@(v) CalcGenericGeometricStatic(...
    cdpr_p,record,data.p(1:3,11),v,geometric_static_mask),...
    [0;0;0],ut.fsolve_options);
    
    data.dt(10) = 1.5; 
    
    data.p(1:3,12)  = cdpr_p.workspace_center+[0.5;0;-0.9];
    data.p(4:6,12)  = fsolve(@(v) CalcGenericGeometricStatic(...
    cdpr_p,record,data.p(1:3,12),v,geometric_static_mask),...
    [0;0;0],ut.fsolve_options);
    
    data.dt(11) = 1.5; 
    
     
    data.p(1:3,13)  = data.p(1:3,1);
    data.p(4:6,13)  = data.p(4:6,1);

    data.dt(12) = 2; 
  
end