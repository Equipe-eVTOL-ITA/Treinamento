# Simulation Parameters
SCREEN_WIDTH = 640
SCREEN_HEIGHT = 480
PIX2M = 0.01  # factor to convert from pixels to meters
M2PIX = 100.0  # factor to convert from meters to pixels

# Sample Time Parameters
FREQUENCY = 60.0  # simulation frequency
SAMPLE_TIME = 1.0 / FREQUENCY  # simulation sample time

# Behavior Parameters
MOVE_FORWARD_TIME = 2.0  # time moving forward before switching to the spiral behavior
MOVE_IN_SPIRAL_TIME = 8.0  # time moving in spiral before switching back to moving forward
GO_BACK_TIME = 0.5  # time going back after hitting a wall
FORWARD_SPEED = 0.5  # default linear speed when going forward
BACKWARD_SPEED = -0.4 # default backward speed when going back after hitting a wall
INITIAL_RADIUS_SPIRAL = 0.2  # initial spiral radius
SPIRAL_FACTOR = 0.3  # factor used to make the spiral grow while the time passes
ANGULAR_SPEED = 1.0  # default angular speed
