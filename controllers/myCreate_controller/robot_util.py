from controller import Robot
from controller import wb, PositionSensor
from controller import Supervisor, Node
from controller import Keyboard
from controller import Motor

from direction import Direction


def wheel_turn(wheel: Motor, direction: Direction, cycles: float):
    '''
    wheel:      Which wheel to rotate
    direction:  Forward or backward
    cycles:     How many cycles to turn, 1 cycle equals 2*PI
    '''
    v = cycles * 6.28
    if direction == Direction.FORWARD:
        c = 1
    elif direction == Direction.BACKWARD:
        c = -1
    elif direction == Direction.STOP:
        c = 0

    value = c * v
    if value > wheel.getMaxVelocity():
        value = wheel.getMaxVelocity()

    wheel.setVelocity(value)