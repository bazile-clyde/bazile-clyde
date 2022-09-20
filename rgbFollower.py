"""Module makes a rover follow a line using a webcam for color detection."""

import asyncio

from viam.robot.client import RobotClient
from viam.rpc.dial import Credentials, DialOptions
from viam.components.base import Base, Vector3
from viam.components.camera import Camera
from viam.services.vision import VisionServiceClient
from enum import Enum, auto


# Copy and paste the following from the Connect tab's Python SDK section
async def connect():
    """
    Connects code to robot.
    """
    creds = Credentials(type="robot-location-secret", payload="e7ztx7s67d4pnnnor54qkn2wlhf58cd7erukcldonha0cc62")
    opts = RobotClient.Options(refresh_interval=0, dial_options=DialOptions(credentials=creds))
    return await RobotClient.at_address("starter-bot-main.k4xl69bmso.local.viam.cloud:8080", opts)


class Direction(Enum):
    IDLE = auto()
    FORWARD = auto()
    LEFT = auto()
    RIGHT = auto()


async def move_to_color(frame, vis, detector_name) -> Direction:
    x, y = frame.size[0], frame.size[1]

    # Crop the image to get only the middle fifth of the top third of the original image
    top_frame = frame.crop((x / 2.5, 0, x / 1.25, y / 3))
    top_det = await vis.get_detections(top_frame, detector_name)
    if top_det:
        return Direction.FORWARD

    # Crop image to get only the left two fifths of the original image
    l_frame = frame.crop((0, 0, x / 2.5, y))
    l_det = await vis.get_detections(l_frame, detector_name)

    # Crop image to get only the right two fifths of the original image
    r_frame = frame.crop((x / 1.25, 0, x, y))
    r_det = await vis.get_detections(r_frame, detector_name)

    if not (r_det or l_det):
        return Direction.IDLE

    # color detected in either right or left; choose larger of the two
    _max = max([r_det, l_det], key=lambda d: (d[0].x_max - d[0].x_min) * (d[0].y_max - d[0].y_min) if d else 0)
    return Direction.LEFT if _max == l_det else Direction.RIGHT


async def main():
    """
    Main line follower function.
    """
    robot = await connect()
    camera = Camera.from_robot(robot, "camera")
    base = Base.from_robot(robot, "base")
    vision = VisionServiceClient.from_robot(robot)

    linear_power = 0.8
    angular_power = 0.65
    try:
        while True:
            frame = await camera.get_image()
            direction = await move_to_color(frame, vision, "green_detector")
            print(f'move:{direction}')

            if Direction.FORWARD is direction:
                await base.set_power(Vector3(y=linear_power), Vector3())
            elif Direction.LEFT is direction:
                await base.set_power(Vector3(), Vector3(z=angular_power))
            elif Direction.RIGHT is direction:
                await base.set_power(Vector3(), Vector3(z=-angular_power))
            else:
                assert Direction.IDLE is direction
                break
    finally:
        await base.stop()
        await robot.close()


if __name__ == "__main__":
    print("Starting up...")
    asyncio.run(main())
    print("Done.")
