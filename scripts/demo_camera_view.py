"""
Live camera viewer — proves the full Gazebo → ROS 2 → Python pipeline works.

What it does:
    1. Spawns an overhead camera into the running Gazebo world (idempotent).
    2. Subscribes to the bridged ROS 2 image topic.
    3. Opens an OpenCV window showing the live camera feed.
    4. Press 'q' or ESC to quit. Press 's' to save a snapshot to disk.

PREREQUISITES:
    1. Gazebo launch is running (Terminal 1):
         ros2 launch ur_simulation_gz ur_sim_control.launch.py ur_type:=ur5

    2. The image bridge is running (Terminal 2, separate from this script):
         ros2 run ros_gz_image image_bridge /camera/image

    The bridge will print 'Created 1 subscriber to /camera/image'. If you
    don't run the bridge, this script will hang waiting for frames.

    3. THIS script (Terminal 3):
         python3 demo_camera_view.py

Topic flow:
    Gazebo camera sensor
        --> /camera/image  (gz-transport, gz.msgs.Image)
        --> [ros_gz_image bridge]
        --> /camera/image  (ROS 2, sensor_msgs/Image)
        --> CameraSubscriber
        --> OpenCV window
"""

import os
import time
from datetime import datetime

import cv2

from spawn_objects import WorldSpawner
from camera import CameraSubscriber


CAMERA_TOPIC = "/camera/image"
SNAPSHOT_DIR = os.path.expanduser("~/ros2_ws/snapshots")


def ensure_camera_exists():
    """Spawn the overhead camera if it isn't already in the world."""
    spawner = WorldSpawner(world_name="empty")
    print("Spawning overhead camera at (0.5, 0, 1.5) pointing down...")
    ok = spawner.spawn_camera(
        x=0.5, y=0.0, z=1.5,
        pitch=1.5708,  # +90° about Y -> camera looks straight down
        yaw=0.0,
        name="overhead_camera",
        topic=CAMERA_TOPIC,
    )
    if not ok:
        print(
            "NOTE: spawn returned False — camera may already exist.\n"
            "      Proceeding anyway. If no frames arrive, remove\n"
            "      'overhead_camera' from the Entity Tree and re-run."
        )
    # Tiny pause so the sensor finishes initializing before we subscribe.
    time.sleep(1.0)


def run_viewer():
    """Open an OpenCV window and stream the camera feed until user quits."""
    os.makedirs(SNAPSHOT_DIR, exist_ok=True)

    with CameraSubscriber(topic=CAMERA_TOPIC) as cam:
        print("\nViewer running.")
        print("  'q' or ESC  : quit")
        print("  's'         : save snapshot to ~/ros2_ws/snapshots/")
        print()

        window = "Overhead Camera (Gazebo)"
        cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)

        last_fps_print = time.time()
        last_frame_count = cam.get_frame_count()

        while True:
            try:
                frame = cam.get_latest_frame()
            except RuntimeError:
                # First-frame race — should not happen since connect() blocks
                # on first frame, but be defensive.
                time.sleep(0.05)
                continue

            cv2.imshow(window, frame)

            # Print FPS once per second so you can see the feed is live.
            now = time.time()
            if now - last_fps_print >= 1.0:
                cur_count = cam.get_frame_count()
                fps = (cur_count - last_frame_count) / (now - last_fps_print)
                print(f"  ~{fps:.1f} fps   (frame #{cur_count})")
                last_fps_print = now
                last_frame_count = cur_count

            # cv2.waitKey is what processes window events. Returns key code,
            # or -1 if no key was pressed within the timeout (in ms).
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):  # 'q' or ESC
                break
            if key == ord('s'):
                fname = datetime.now().strftime("snapshot_%Y%m%d_%H%M%S.png")
                path = os.path.join(SNAPSHOT_DIR, fname)
                cv2.imwrite(path, frame)
                print(f"  Saved: {path}")

        cv2.destroyAllWindows()


def main():
    print("=" * 60)
    print(" Overhead Camera Viewer")
    print("=" * 60)
    ensure_camera_exists()
    run_viewer()
    print("Done.")


if __name__ == "__main__":
    main()