CONTROL = {
  WBC = {
    kp_body_Cartesian = 100,
    kd_body_Cartesian = 10,
    kp_body_rotation = 1000,
    kd_body_rotation = 100,
    kp_foot_Cartesian = 100,
    kd_foot_Cartesian = 10,
    
    force_loss = 1.0,
    acc_loss = 0.1,
  },
  
  MPC = {
    update_period = 0.03,
    pred_interval = 0.02,
    pred_horizon = 5,
    
    pos_w = 10.,
    vel_w = 0.1,
    rot_w = 100.,
    rot_vel_w = 1,
    force_w = 1e-6,
  
    f_z_max = 100,
    f_z_min = 1,
  },
  
  TRAJECTORY = {
    traj_len = 1.5
  },
  
  control_period = 0.001,
}

