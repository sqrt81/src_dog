CONTROL = {
  WBC = {
    kp_body_Cartesian = 0.00100,
    kd_body_Cartesian = 0.0010,
    kp_body_rotation = 0.00200,
    kd_body_rotation = 0.0020,
    kp_foot_Cartesian = 0.00100,
    kd_foot_Cartesian = 0.0010,
    
    force_loss = 1000.0,
    acc_loss = 0.001,
  },
  
  MPC = {
    update_period = 0.01,
    pred_interval = 0.02,
    pred_horizon = 5,
    
    pos_w = 1.,
    vel_w = 0.01,
    rot_w = 0.01,
    rot_vel_w = 0.01,
    force_w = 1e-6,
  
    f_z_max = 100,
    f_z_min = 1,
  },
  
  control_period = 0.001,
}

