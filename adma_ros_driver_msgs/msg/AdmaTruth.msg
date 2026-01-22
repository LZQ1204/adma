std_msgs/Header header
float64 actual_spd        # m/s, from ins_vel_frame sqrt(x^2 + y^2)
float64 yaw_rate          # deg/s, from rate_hor.z
float64 lat_accel         # g, from acc_hor.y (lateral)
float64 long_accel        # g, from acc_hor.x (longitudinal)
uint8 yaw_rate_sign       # 0 = non-negative, 1 = negative
