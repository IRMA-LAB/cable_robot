classdef PlatformVar
  properties
    position;
    orientation;
    quaternion;
    
    rot_mat;
    pose;
    pose_q;
    
    pos_PG_glob;
    pos_OG_glob;
    
    velocity;
    orientation_deriv;
    quaternion_deriv;
    angular_vel;
    vel_OG_glob;
    H_mat;
    
    acceleration;
    orientation_deriv_2;
    quaternion_deriv_2;
    angular_acc;
    acc_OG_glob;
    H_mat_deriv;

  end
  methods
    function obj = UpdatePose(obj,pos,orient,ang_type)
      obj.position = pos;
      switch (ang_type)
        case RotationParametrizations.EULER_ZYZ
          obj.rot_mat = RotZYZ(orient);
          obj.quaternion = Rot2Quat(obj.rot_mat);
          obj.orientation = orient;
          obj.pose = [pos;orient];
          obj.pose_q = [pos;obj.quaternion];
        case RotationParametrizations.TAYT_BRYAN
          obj.rot_mat = RotXYZ(orient);
          obj.orientation = orient;
          obj.pose = [pos;orient];
          obj.quaternion = Rot2Quat(obj.rot_mat);
          obj.pose_q = [pos;obj.quaternion];
        case RotationParametrizations.RPY
          obj.rot_mat = RotRPY(orient);
          obj.orientation = orient;
          obj.pose = [pos;orient];
          obj.quaternion = Rot2Quat(obj.rot_mat);
          obj.pose_q = [pos;obj.quaternion];
        case RotationParametrizations.TILT_TORSION
          obj.rot_mat = RotTiltTorsion(orient);
          obj.orientation = orient;
          obj.pose = [pos;orient];
          obj.quaternion = Rot2Quat(obj.rot_mat);
          obj.pose_q = [pos;obj.quaternion];
        case RotationParametrizations.QUATERNION
          obj.rot_mat = Quat2Rot(orient);
          obj.quaternion = orient;
          obj.pose_q = [pos;orient];
          %obj.orient = ??;
          %obj.pose = ??;
      end
    end
    function obj = UpdateVelocity(obj,vel,orient_d,ang_type)
      obj.velocity = vel;
      switch (ang_type)
        case RotationParametrizations.EULER_ZYZ
          obj.orientation_deriv = orient_d;
          %obj.quaternion_deriv = ??;
          obj.H_mat = HtfZYZ(obj.orientation);         
        case RotationParametrizations.TAYT_BRYAN
          obj.orientation_deriv = orient_d;
          %obj.quaternion_deriv = ??;
          obj.H_mat = HtfTaytBryan(obj.orientation);
        case RotationParametrizations.RPY
          obj.orientation_deriv = orient_d;
          %obj.quaternion_deriv = ??;
          obj.H_mat = HtfRPY(obj.orientation);
        case RotationParametrizations.TILT_TORSION
          obj.orientation_deriv = orient_d;
          %obj.quaternion_deriv = ??;
          obj.H_mat = HtfTiltTorsion(obj.orientation);
        case RotationParametrizations.QUATERNION
          %obj.orientation_deriv = orient_d;
          obj.quaternion_deriv = orient_d;
          obj.H_mat = HtfQuaternion(obj.orientation);
      end
      obj.angular_vel = obj.H_mat*orient_d;
    end
    function obj = UpdateAcceleration(obj,acc,orient_d_2,ang_type)
      obj.velocity = acc;
      switch (ang_type)
        case RotationParametrizations.EULER_ZYZ
          obj.orientation_deriv_2 = orient_d_2;
          %obj.quaternion_deriv = ??;
          obj.H_mat_deriv = DHtfZYZ(obj.orientation,obj.orientation_deriv)
        case RotationParametrizations.TAYT_BRYAN
          obj.orientation_deriv_2 = orient_d_2;
          %obj.quaternion_deriv = ??;
          obj.H_mat_deriv = DHtfTaytBryan(obj.orientation,obj.orientation_deriv);
        case RotationParametrizations.RPY
          obj.orientation_deriv_2 = orient_d_2;
          %obj.quaternion_deriv = ??;
          obj.H_mat_deriv = DHtfRPY(obj.orientation,obj.orientation_deriv);
        case RotationParametrizations.TILT_TORSION
          obj.orientation_deriv_2 = orient_d_2;
          %obj.quaternion_deriv = ??;
          obj.H_mat_deriv = DHtfTiltTorsion(obj.orientation,obj.orientation_deriv);
        case RotationParametrizations.QUATERNION
          %obj.orientation_deriv_2 = ??;
          obj.quaternion_deriv_2 = orient_d_2;
          obj.H_mat_deriv = DHtfQuaternion(obj.orientation,obj.orientation_deriv);
      end
      obj.angular_acc = obj.H_mat*orient_d_2 +...
            obj.H_mat_deriv*obj.orient_deriv;
    end
  end
end