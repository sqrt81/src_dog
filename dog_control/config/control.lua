CONTROL = {
  WBC = {
    kp_body_Cartesian = 100,
    kd_body_Cartesian = 10,
    kp_body_rotation = 100,
    kd_body_rotation = 10,
    kp_foot_Cartesian = 10,
    kd_foot_Cartesian = 5,
    
    force_loss = 1,
    acc_loss = 0.1,
  },
  
  MPC = {
    update_period = 0.02,
    
    pred_interval = 0.02,
    pred_horizon = 5,
    
    total_inertial_x = 150e-3,
    total_inertial_y = 150e-3,
    total_inertial_z = 200e-3,
    
    pos_w = 10.,
    vel_w = 0.1,
    rot_w = 100.,
    rot_vel_w = 0.1,
    force_w = 1e-6,
  
    f_z_max = 100,
    f_z_min = 5,
  },
  
  ExtendedMPC = {
    pred_interval_1 = 0.02,
    pred_interval_2 = 0.05,
    pred_interval_3 = 0.1,
    pred_interval_4 = 0.2,
    pred_interval_5 = 0.2,
    pred_interval_6 = 0.2,
  },
  
  TRAJECTORY = {
    traj_len = 1.5
  },
  
  control_period = 0.001,
}

