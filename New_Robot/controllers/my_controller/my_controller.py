from controller import Robot

# Create robot instance
robot = Robot()

# Time step (ms)
timestep = int(robot.getBasicTimeStep())

# Get motors
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf'))  # Velocity control mode
right_motor.setPosition(float('inf'))

left_motor.setVelocity(2.0)   # Move forward
right_motor.setVelocity(2.0)

# Get camera
camera = robot.getDevice("camera")
camera.enable(timestep)

while robot.step(timestep) != -1:
    # Get camera image
    image = camera.getImage()
    if image:
        width = camera.getWidth()
        height = camera.getHeight()
        print(f"Camera image received: {width}x{height}")

