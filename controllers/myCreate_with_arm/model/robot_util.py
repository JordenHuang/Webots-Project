from pycreate2 import Create2
from . import direction


motor_left_velocity = 0
motor_right_velocity = 0

def drive_my_create(bot: Create2, direction: direction.Direction):
    # velocity in mm/s, in the range[-500, 500]
    velocity = 100
    if direction == direction.STOP:
        bot.drive_stop()
    elif direction == direction.FORWARD:
        bot.drive_direct(velocity, velocity)
    elif direction == direction.BACKWARD:
        bot.drive_direct(-velocity, -velocity)
    elif direction == direction.LEFT:
        bot.drive_direct(-velocity, velocity)
    elif direction == direction.RIGHT:
        bot.drive_direct(velocity, -velocity)
    elif direction == direction.FORWARD_LEFT:
        bot.drive_direct(velocity//2, velocity)
    elif direction == direction.FORWARD_RIGHT:
        bot.drive_direct(velocity, velocity//2)
    elif direction == direction.BACKWARD_LEFT:
        bot.drive_direct(-velocity//2, -velocity)
    elif direction == direction.BACKWARD_RIGHT:
        bot.drive_direct(-velocity, -velocity//2)
    else:
        # Unknown direction
        bot.drive_stop()

def wheel_turn(bot: Create2, wheel: str, direction: direction.Direction, cycles: float):
    '''
    DON'T USE THIS

    wheel:      Which wheel to rotate
    direction:  Forward or backward
    cycles:     How many cycles to turn, 1 cycle equals 2*PI
    '''
    global motor_left_velocity, motor_right_velocity
    MAX_VELOCITY = 500

    v = cycles * MAX_VELOCITY
    if direction == direction.FORWARD:
        c = 1
    elif direction == direction.BACKWARD:
        c = -1
    elif direction == direction.STOP:
        c = 0

    value = int(c * v)
    if value > MAX_VELOCITY:
        value = MAX_VELOCITY
    print(value)

    if wheel == "left":
        bot.drive_direct(value, motor_right_velocity)
        motor_left_velocity = value
    elif wheel == "right":
        bot.drive_direct(motor_left_velocity, value)
        motor_right_velocity = value