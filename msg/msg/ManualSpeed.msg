# Simple Stepper Speed Message
# that multidriver should listen to
# in order to ascertain
#Header header

# Stepper speeds in steps per second
float64[] speed

# Optional Profile Set Section
uint8[]   motor_indice
float64[] max_speed # steps/s
float64[] accel # steps/s^2
float64[] decel # steps/s^2
