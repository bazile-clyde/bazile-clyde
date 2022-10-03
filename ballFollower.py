"""Module makes a rover follow a line using a webcam for color detection."""

import asyncio
from dataclasses import dataclass

from viam.robot.client import RobotClient
from viam.rpc.dial import Credentials, DialOptions
from viam.components.base import Base
from viam.components.camera import Camera
from viam.services.vision import VisionServiceClient, VisModelConfig, VisModelType
from enum import Enum, auto


async def connect():
    creds = Credentials(type="robot-location-secret", payload="e7ztx7s67d4pnnnor54qkn2wlhf58cd7erukcldonha0cc62")
    opts = RobotClient.Options(refresh_interval=0, dial_options=DialOptions(credentials=creds))
    return await RobotClient.at_address("starter-bot-main.k4xl69bmso.local.viam.cloud:8080", opts)


class Direction(Enum):
    STOP = auto()
    FORWARD = auto()
    LEFT = auto()
    RIGHT = auto()


async def move_to_ball(vis, midpoint) -> Direction:
    async def get_object(name, detector):
        for detection in await vis.get_detections_from_camera("camera", detector):
            if detection.class_name == name:
                return detection
        return None

    ball = await get_object("red", "ball_detector")
    line = await get_object("light-blue", "line_detector")

    if not (ball and line):
        return Direction.STOP

    if ball.y_min < line.y_min and ball.y_max < line.y_max:
        return Direction.STOP

    centerX = ball.x_min + ball.x_max / 2
    if centerX < midpoint - midpoint / 6:
        return Direction.LEFT
    if centerX > midpoint + midpoint / 6:
        return Direction.RIGHT
    else:
        return Direction.FORWARD


async def init_classifier(vis):
    params = {
        "detect_color": "#77181a",  # red
        "tolerance": 0.05,
        "segment_size": 1000
    }

    await vis.add_detector(VisModelConfig(name="ball_detector", type=VisModelType.DETECTOR_COLOR, parameters=params))
    assert "ball_detector" in await vis.get_detector_names()

    params['detect_color'] = '#2e4c7d'  # light-blue
    await vis.add_detector(VisModelConfig(name="line_detector", type=VisModelType.DETECTOR_COLOR, parameters=params))
    assert "line_detector" in await vis.get_detector_names()


@dataclass
class Op:
    direction: Direction
    power: int
    velocity: int


async def pop_and_reverse_all(stack, base):
    print("going back to start")
    while stack:
        await pop_and_reverse(stack, base)


async def pop_and_reverse(stack, base):
    op = stack.pop()
    if Direction.FORWARD == op.direction:
        await base.move_straight(-op.power, op.velocity)
    else:  # Direction.LEFT == op.direction or Direction.RIGHT == op.direction:
        await base.spin(-op.power, op.velocity)


async def main():
    """
    Main line follower function.
    """
    robot = await connect()
    base = Base.from_robot(robot, "base")
    vision = VisionServiceClient.from_robot(robot)
    await init_classifier(vision)

    camera = Camera.from_robot(robot, "camera")
    frame = await camera.get_image()
    midpoint = frame.size[0]/2

    stack = []
    try:
        angle = 10  # when turning, spin the motor this much
        straight_num = 300      	# when going straight, spin motor this much
        vel = 500
        stop_count = 10  # start returning home after this many consecutive stops in a row
        while True:
            direction = await move_to_ball(vision, midpoint)
            print(f'move:{direction}')

            # we do this incrementally just in case we need to stop returning home and start pursing another object
            if Direction.STOP is direction:
                await base.stop()
                if not stop_count:
                    if stack:
                        await pop_and_reverse(stack, base)
                else:
                    stop_count -= 1
                continue  # no need to append Direction.STOP to stack

            async def move_and_push(move, angle):
                await move(angle, vel)
                stack.append(Op(direction, angle, vel))

            if not Direction.FORWARD:  # either LEFT or RIGHT; do turn first
                await move_and_push(base.spin, angle if Direction.LEFT else -angle)  # else Direction.RIGHT
            await move_and_push(base.move_straight, straight_num)
            stop_count = 10  # reset stop count
    finally:
        await base.stop()
        await pop_and_reverse_all(stack, base)
        await robot.close()


if __name__ == "__main__":
    print("Starting up...")
    asyncio.run(main())
    print("Done.")
