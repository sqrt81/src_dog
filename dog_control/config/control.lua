CONTROL = {
  WBC = {
    kp_body_Cartesian = 1,
    kd_body_Cartesian = 0.1,
    kp_body_rotation = 0,
    kd_body_rotation = 0,
    kp_foot_Cartesian = 10,
    kd_foot_Cartesian = 2,
    
    force_loss = 1,
    acc_loss = 0.01,
  },
  
  MPC = {
    update_period = 0.01,
    
    pred_interval = 0.02,
    pred_horizon = 5,
    
    total_inertial_x = 150e-3,
    total_inertial_y = 150e-3,
    total_inertial_z = 200e-3,
    
    pos_w = 0.1,
    vel_w = 1,
    rot_w = 1,
    rot_vel_w = 10,
    force_w = 1e-6,
  
    f_z_max = 100,
    f_z_min = 5,
  },
  
  ExtendedMPC = {
    force_diff_w = 0,
    pred_interval_1 = 0.02,
    pred_interval_2 = 0.02,
    pred_interval_3 = 0.05,
    pred_interval_4 = 0.05,
    pred_interval_5 = 0.1,
    pred_interval_6 = 0.1,
  },
  
  TRAJECTORY = {
    traj_len = 1.5
  },
  
  control_period = 0.001,
}

