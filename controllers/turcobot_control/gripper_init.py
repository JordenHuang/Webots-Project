from controller import Robot
from the_constant import PI

gripper_right_base_outer = None
gripper_right_outer_paddle = None
gripper_right_inner = None
gripper_left_base_outer = None
gripper_left_outer_paddle = None
gripper_left_inner = None

def gripper_init(turtlebot: Robot):
    global gripper_right_base_outer
    global gripper_right_outer_paddle
    global gripper_right_inner
    global gripper_left_base_outer
    global gripper_left_outer_paddle
    global gripper_left_inner

    gripper_right_base_outer = turtlebot.getDevice("gripper_right::base_outer::rotational_motor")
    gripper_right_outer_paddle = turtlebot.getDevice( "gripper_right::outer_paddle::rotational_motor")
    gripper_right_inner = turtlebot.getDevice("gripper_right::base_inner::rotational_motor")

    gripper_left_base_outer = turtlebot.getDevice("gripper_left::base_outer::rotational_motor")
    gripper_left_outer_paddle = turtlebot.getDevice( "gripper_left::outer_paddle::rotational_motor")
    gripper_left_inner = turtlebot.getDevice("gripper_left::base_inner::rotational_motor")


def mycobot_gripper_send_angle(degree: int, speed=0.007):
    rad_right = degree * (PI / 180)  # Convert degrees to radians
    current_angle_right = gripper_right_base_outer.getTargetPosition()  # Get current joint angles

    rad_left = (-degree) * (PI / 180)  # Convert degrees to radians
    current_angle_left = gripper_left_base_outer.getTargetPosition()  # Get current joint angles

    flag = False
    while True:  # Loop until close enough
        flag = False
        # Right
        if abs(current_angle_right - rad_right) > 0.01:
            diff = rad_right - current_angle_right
            step = speed if abs(diff) > speed else abs(diff)  # Step should not exceed remaining distance
            current_angle_right += step * (1 if diff > 0 else -1)  # Move in the correct direction

            gripper_right_base_outer.setPosition(current_angle_right)  # Apply new position
            gripper_right_outer_paddle.setPosition(-current_angle_right)  # Apply new position
            gripper_right_inner.setPosition(current_angle_right)  # Apply new position
        else:
            flag = True

        # Left
        if abs(current_angle_left - rad_left) > 0.01:
            diff = rad_left - current_angle_left
            step = speed if abs(diff) > speed else abs(diff)  # Step should not exceed remaining distance
            current_angle_left += step * (1 if diff > 0 else -1)  # Move in the correct direction

            gripper_left_base_outer.setPosition(current_angle_left)  # Apply new position
            gripper_left_outer_paddle.setPosition(-current_angle_left)  # Apply new position
            gripper_left_inner.setPosition(current_angle_left)  # Apply new position
        else:
            if flag == True:
                break
        
        myCobot.step(2*TIMESTEP)  # Small delay to control speed