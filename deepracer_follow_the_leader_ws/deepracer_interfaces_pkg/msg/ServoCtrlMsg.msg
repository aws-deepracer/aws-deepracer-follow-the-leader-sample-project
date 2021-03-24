# Custom message with throttle and steering angle values.
#
# This message is used to send motor throttle and servo steering angle
# ratios with respect to the device calibration. It can also be used to
# send raw PWM values for angle and throttle.
float32 angle       # Angle ratio w.r.t steering_max/steering_min
                    # in calibration file or raw PWM value to be sent
                    # to the servo.
float32 throttle    # Throttle ratio w.r.t speed_max/speed_min
                    # in calibration file or raw PWM value to be sent
                    # to the motor.   