from controller import Robot
from controller import Keyboard
import sys
import cv2
import numpy as np
import socket
from random import randint



# Create the robot instance
myCreate = Robot()

# get the time step of the current world.
TIMESTEP = int(myCreate.getBasicTimeStep())

keyboard = Keyboard()
keyboard.enable(TIMESTEP)

# ========== MyCreate ==========
MAX_SPEED  = 16
NULL_SPEED = 0
HALF_SPEED = 8
MIN_SPEED  = -16

WHEEL_RADIUS       = 0.031
AXLE_LENGTH        = 0.271756 # 0.235
ENCODER_RESOLUTION = 507.9188

TO_LEFT  = -np.pi / 2
TO_RIGHT = np.pi / 2
GO_FRONT = 0.1
GO_BACK  = -0.1

left_motor = myCreate.getDevice("left wheel motor")
right_motor = myCreate.getDevice("right wheel motor")
left_motor.setPosition(float('+inf'));
right_motor.setPosition(float('+inf'));
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

left_position_sensor = myCreate.getDevice("left wheel sensor")
right_position_sensor = myCreate.getDevice("right wheel sensor")
left_position_sensor.enable(TIMESTEP)
right_position_sensor.enable(TIMESTEP)

camera_left = myCreate.getDevice("camera_left")
camera_right = myCreate.getDevice("camera_right")
camera_left.enable(TIMESTEP)
camera_right.enable(TIMESTEP)

bumpers = {
    "bumper_left" : None,
    "bumper_right": None,
}
for name in bumpers.keys():
    bumpers[name] = myCreate.getDevice(name)
    bumpers[name].enable(TIMESTEP)
def is_there_a_collision_at_left() -> bool:
    return bumpers["bumper_left"].getValue() != 0.0

def is_there_a_collision_at_right() -> bool:
    return bumpers["bumper_right"].getValue() != 0.0

def step():
    if myCreate.step(TIMESTEP) == -1:
        sys.exit(0)

def stop():
    left_motor.setVelocity(-NULL_SPEED)
    right_motor.setVelocity(-NULL_SPEED)

def turn(angle: float):
    l_offset = left_position_sensor.getValue()
    r_offset = right_position_sensor.getValue()
    neg = -1.0 if angle < 0.0 else 1.0
    vel = neg * np.pi * 1
    left_motor.setVelocity(vel)
    right_motor.setVelocity(-vel)
    while True:
        step()
        l = left_position_sensor.getValue() - l_offset
        r = right_position_sensor.getValue() - r_offset
        dl = l * WHEEL_RADIUS                        # distance covered by left wheel in meter
        dr = r * WHEEL_RADIUS                        # distance covered by right wheel in meter
        orientation = neg * (dl - dr) / AXLE_LENGTH  # delta orientation in radian
        # print(orientation, angle)
        # 希望多轉一點
        if abs(angle) - abs(orientation) < -0.08:
            break
    stop()
    step()

# Turn 90 degrees
def my_turn(angle: float):
    neg = -1.0 if angle < 0.0 else 1.0
    vel = neg * np.pi * 1
    left_motor.setVelocity(vel)
    right_motor.setVelocity(-vel)
    myCreate.step(TIMESTEP * 74)
    # stop()
    # i = (np.pi * AXLE_LENGTH / 2) / (WHEEL_RADIUS * np.pi * 2)
    # neg = -1.0 if angle < 0.0 else 1.0
    # print(i)
    # left_motor.setVelocity(neg * np.pi * 2)
    # right_motor.setVelocity(-neg * np.pi * 2)
    # c = 0
    # while c <= i:
    #     myCreate.step(TIMESTEP)
    #     c += 1 / TIMESTEP
    # stop()
    return

def go(distance_in_m: float):
    # print(f"Go {distance_in_m}m")
    l_offset = left_position_sensor.getValue()
    neg = -1.0 if distance_in_m < 0.0 else 1.0
    vel = neg * np.pi * 1
    left_motor.setVelocity(vel)
    right_motor.setVelocity(vel)
    flag = False
    while True:
        step()
        l = left_position_sensor.getValue() - l_offset
        dl = l * WHEEL_RADIUS                        # distance covered by left wheel in meter
        # print(f"dl: {dl}")
        if abs(distance_in_m) - abs(dl) <= 0.001:
            break

        if is_there_a_collision_at_left() or is_there_a_collision_at_right():
            print("[bump] collision")
            flag = True
            break
    stop()
    step()

    if flag:
        distance_in_m = dl
        left_motor.setVelocity(-vel)
        right_motor.setVelocity(-vel)
        l_offset = left_position_sensor.getValue()
        while True:
            step()
            l = left_position_sensor.getValue() - l_offset
            dl = l * WHEEL_RADIUS                        # distance covered by left wheel in meter
            # print(f"dl: {dl}")
            if abs(distance_in_m) - abs(dl) <= 0.001:
                break
        stop()
        step()
        return -1
    else:
        return 1


# ========== MyCobot ==========
# joints = []
# for i in range(6):
#     joints.append(myCreate.getDevice(f"joint{i}_rotational_motor"))

# PI = 3.14159
# def mycobot_send_angles(degrees: list, speed=0.05):
#     rad = [d * (PI / 180) for d in degrees]  # Convert degrees to radians
#     current_angles = [j.getTargetPosition() for j in joints]  # Get current joint angles

#     while any(abs(curr - target) > 0.01 for curr, target in zip(current_angles, rad)):  # Loop until close enough
#         for i in range(len(joints)):
#             diff = rad[i] - current_angles[i]
#             step = speed if abs(diff) > speed else abs(diff)  # Step should not exceed remaining distance
#             current_angles[i] += step * (1 if diff > 0 else -1)  # Move in the correct direction
#             joints[i].setPosition(current_angles[i])  # Apply new position

#         # time.sleep(0.01)  # Control loop timing
#         myCreate.step(TIMESTEP)  # Small delay to control speed

# ==============================

# degrees = [0, -135, 150, -125, 90, 0]
# mycobot_send_angles(degrees)

# print("Try to connect to server...")
# clientsocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# clientsocket.connect(('localhost', 8989))
# print("Connected")


# Intrinsic parameters
width = camera_left.getWidth()
height = camera_left.getHeight()
field_of_view = camera_left.getFov()
focal_length = (width/2) / np.tan(field_of_view/2)
fx, fy = focal_length, focal_length
cx = width / 2.0
cy = height / 2.0
b = 0.08 # m
print(f"width: {width}, height: {height}, focal length: {focal_length}")

K_left = np.array([[fx, 0, cx],
                [0, fy, cy],
                [0, 0, 1]], dtype=np.float64)
D_left = np.zeros((4, 1), dtype=np.float64) # Assuming no distortion

K_right = np.array([[fx, 0, cx],
                    [0, fy, cy],
                    [0, 0, 1]], dtype=np.float64)
D_right = np.zeros((4, 1), dtype=np.float64) # Assuming no distortion

# Get relative transform from Webots (example, depends on your robot's structure)
# This is simplified. In Webots, you'd get the World-to-Camera transform for each camera,
# then calculate CameraLeft-to-CameraRight transform.
# Let's say `b` is your baseline (distance between cameras along X-axis for standard stereo)
# For example, if left camera is at (0,0,0) in robot base, and right at (b,0,0) in robot base.
# T: translation vector from left camera to right camera
# R: rotation matrix from left camera to right camera (usually identity for perfectly aligned)

# Example (you need to get these accurately from your Webots model!)
# For a typical horizontal stereo rig, R is identity and T is (baseline, 0, 0)
R_left_to_right = np.eye(3, dtype=np.float64) # Identity matrix if cameras are perfectly aligned
T_left_to_right = np.array([0, -b, 0], dtype=np.float64) # b is your baseline in meters

# R1, R2: Rectification transforms for left and right cameras
# P1, P2: New projection matrices for rectified cameras (P_rect = K_new @ [I|0] or K_new @ [I|T'])
# Q_reproj: Reprojection matrix for reprojectImageTo3D (this is what you need!)
R1, R2, P1, P2, Q_reproj, validPixROI1, validPixROI2 = cv2.stereoRectify(
    cameraMatrix1=K_left,
    distCoeffs1=D_left,
    cameraMatrix2=K_right,
    distCoeffs2=D_right,
    imageSize=(width, height),
    R=R_left_to_right,
    T=T_left_to_right,
    flags=cv2.CALIB_ZERO_DISPARITY, # Ensures principal points are aligned horizontally
    alpha=0 # Scales the images, 0=no black borders, 1=all original pixels
)

# Compute undistortion and rectification maps
map1_left, map2_left = cv2.initUndistortRectifyMap(K_left, D_left, R1, P1, (width, height), cv2.CV_16SC2)
map1_right, map2_right = cv2.initUndistortRectifyMap(K_right, D_right, R2, P2, (width, height), cv2.CV_16SC2)


counter = 0
back_counter = 0
update = True
# Main loop
while myCreate.step(TIMESTEP) != -1:
    WHEEL_MAX_SPEED = 6.28
    FORWARD_RATIO = 1 # 0.75

    update = True
    key = keyboard.getKey()
    if key == ord('W'):
        go(GO_FRONT)
        print("GO_FRONT")
    elif key == ord('S'):
        go(GO_BACK)
        print("GO_BACK")
    elif key == ord('A'):
        # turn(TO_LEFT)
        my_turn(TO_LEFT)
        print("TO_LEFT")
    elif key == ord('D'):
        # turn(TO_RIGHT)
        my_turn(TO_RIGHT)
        print("TO_RIGHT")
    else:
        # update = False
        stop()



    # Inside your main loop:
    camRawLeft = camera_left.getImage()
    camRawRight = camera_right.getImage()
    width = camera_left.getWidth()
    height = camera_left.getHeight()

    # Convert to a NumPy array, reshape, and convert RGB to BGR for OpenCV processing
    # Webots RGBA to OpenCV BGR
    image_np_left = np.frombuffer(camRawLeft, dtype=np.uint8).reshape((height, width, 4))
    camImageLeft_bgr = image_np_left[:, :, :3]

    image_np_right = np.frombuffer(camRawRight, dtype=np.uint8).reshape((height, width, 4))
    camImageRight_bgr = image_np_right[:, :, :3]

    # Display raw BGR images (optional, for debugging)
    # cv2.imshow('Left Raw', camImageLeft_bgr)
    # cv2.imshow('Right Raw', camImageRight_bgr)

    # --- Rectify the images ---
    rectified_left = cv2.remap(camImageLeft_bgr, map1_left, map2_left, cv2.INTER_LINEAR)
    rectified_right = cv2.remap(camImageRight_bgr, map1_right, map2_right, cv2.INTER_LINEAR)

    # Display rectified images (for debugging)
    # cv2.imshow('Left Rectified', rectified_left)
    # cv2.imshow('Right Rectified', rectified_right)

    # Convert to grayscale for stereo matching
    gray_left_rectified = cv2.cvtColor(rectified_left, cv2.COLOR_BGR2GRAY)
    gray_right_rectified = cv2.cvtColor(rectified_right, cv2.COLOR_BGR2GRAY)

    # --- Compute Disparity on Rectified Images ---
    # (Your StereoBM/SGBM settings seem reasonable, tune numDisparities and blockSize)
    # numDisparities must be divisible by 16. max_disparity - min_disparity
    # blockSize should be odd (3, 5, 7, ..., 15)
    # stereo = cv2.StereoSGBM_create(
    #     minDisparity=0, # or a small negative if your setup can have it
    #     numDisparities=64, # must be divisible by 16
    #     blockSize=15,
    #     P1=8 * 3 * 15**2, # SGBM parameters, needs tuning
    #     P2=32 * 3 * 15**2,
    #     disp12MaxDiff=1,
    #     uniquenessRatio=10,
    #     speckleWindowSize=100,
    #     speckleRange=32,
    #     preFilterCap=63,
    #     # mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY # Or default
    # )
    # 保持 blockSize 為 5~15 之間的奇數，numDisparities 為 16 的倍數
    window_size = 5
    stereo = cv2.StereoSGBM_create(
        minDisparity=0,
        # numDisparities=128*3,  # 建議加大範圍再觀察
        numDisparities=320,  # 建議加大範圍再觀察
        blockSize=window_size,         # 嘗試小一點的 blockSize (5~11)
        P1=8 * 3 * window_size**2,
        P2=32 * 3 * window_size**2,
        disp12MaxDiff=1,
        uniquenessRatio=5,
        speckleWindowSize=50,
        speckleRange=2,
        preFilterCap=63,
        # mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY  # 增強一致性
    )
    disparity = stereo.compute(gray_left_rectified, gray_right_rectified).astype(np.float32) / 16.0

    # Normalize disparity for display (optional, but good for visualization)
    # disparity_display = cv2.normalize(disparity, disparity, alpha=255,
    #                                 beta=0, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    # cv2.imshow('Disparity Map', disparity_display)
    disparity_display = cv2.normalize(disparity.copy(), None, alpha=0, beta=255,
                                    norm_type=cv2.NORM_MINMAX)
    disparity_display = np.uint8(disparity_display)
    # cv2.imshow('Disparity Map', disparity_display)
    # print("Disparity min/max:", np.min(disparity), np.max(disparity))
    # cv2.imwrite("disparity_debug.png", disparity_display)

    r = 2
    combined = np.hstack([
        cv2.resize(rectified_left, (width//r, height//r)),
        cv2.resize(rectified_right, (width//r, height//r)),
        cv2.resize(cv2.cvtColor(disparity_display, cv2.COLOR_GRAY2BGR), (width//r, height//r))
    ])
    cv2.imshow("Stereo Pipeline Debug", combined)
    cv2.imwrite("StereoPipelineDebug.png", combined)


    
    # --- Reproject to 3D using the CORRECT Q_reproj matrix ---
    # The Q_reproj from cv2.stereoRectify is critical here.
    threeDPoint = cv2.reprojectImageTo3D(disparity, Q_reproj)

    # threeDPoint now contains X, Y, Z coordinates in the left camera's rectified coordinate system.
    # The Z value directly corresponds to depth.
    # You can now extract the depth map if needed:
    depth_map = threeDPoint[:, :, 2] # Z-coordinate is depth

    # You can also visualize the depth map (normalize it for display)
    # depth_display = cv2.normalize(depth_map, depth_map, alpha=255,
    #                             beta=0, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    # cv2.imshow('Depth Map', depth_display)

    # The individual depth calculation you had will now work correctly for a specific pixel:
    # If you pick a point (u,v) from the *rectified* left image
    # disparity_at_point = disparity[v, u]
    # actual_depth_at_point = (P1[0,0] * b) / disparity_at_point # P1[0,0] is the rectified focal length
    # disparity_at_point = disparity[468, 1019]
    # actual_depth_at_point = (P1[0,0] * b) / disparity_at_point # P1[0,0] is the rectified focal length
    # print(f"actual_depth_at_point: {actual_depth_at_point}")

    # u, v = 1019, 417
    # u, v = 410, 350 
    # d = disparity[v, u]
    # if d > 0:
    #     Z = (P1[0,0] * b) / d
    #     print(f"Depth at ({u},{v}): {Z:.2f} m")
    # else:
    #     print("No valid disparity at this point.")

    # valid = (disparity > 0).astype(np.uint8) * 255
    # cv2.imshow("Valid Disparity", valid)


    # print(f"Baseline (B): {b} m")
    # print(f"Focal length (f): {focal_length} pixels")
    # print(f"Disparity at ({u},{v}): {d} pixels")

    # x, y = 350, 410
    # x, y = 391, 391
    # x, y = 354, 210
    # x, y = 583, 285
    x, y = 534, 412
    d = disparity[y, x]
    print(f"Disparity at ({x},{y}): {d}")
    point = depth_map[y, x]
    print(f"Depth at ({x},{y}): {point:.2f} m")

    
    # Remember to also handle cv2.waitKey(1) and cv2.destroyAllWindows() properly in your main loop's exit condition.
    cv2.waitKey(1)

    '''
    # Convert to a NumPy array, reshape, and convert RGB to BGR
    image_np = np.frombuffer(camRawLeft, dtype=np.uint8).reshape((height, width, 4))  # RGBA
    camImageLeft = image_np[:, :, :3]  # Drop alpha channel
    # camImageLeft = cv2.cvtColor(camImageLeft, cv2.COLOR_BGR2RGB)
    image_np2 = np.frombuffer(camRawRight, dtype=np.uint8).reshape((height, width, 4))  # RGBA
    camImageRight = image_np2[:, :, :3]  # Drop alpha channel
    # camImageRight = cv2.cvtColor(camImageRight, cv2.COLOR_BGR2RGB)

    cv2.imshow('Left' , camImageLeft)
    cv2.imshow('Right', camImageRight)
    cv2.waitKey(1)

    gray_left = cv2.cvtColor(camImageLeft, cv2.COLOR_BGR2GRAY)
    gray_right = cv2.cvtColor(camImageRight, cv2.COLOR_BGR2GRAY)

    # stereo = cv2.StereoBM_create(numDisparities=64, blockSize=15)
    stereo = cv2.StereoSGBM_create(numDisparities=64, blockSize=15)
    disparity = stereo.compute(gray_left, gray_right).astype(np.float32) / 16.0

    cv2.imshow('disp', disparity)
    # cv2.stereoRectify()

    Y = 118 / (189 - 133)
    # print(camera_left.getFocalDistance())
    b = 0.1 # m
    f = 320 # 焦距
    # depth = b*f / (disparity+1e-6)
    depth = b*f / (242 - 137) # kukabox bottom left corner
    # depth = b*f / (110 - 87) # third brown floor left corner
    print(f"{depth}m")

    #This transformation matrix is from the openCV documentation, didn't seem to work for me. 
    Q = np.float32([[1,0,0,-width/2.0],
                    [0,-1,0,height/2.0],
                    [0,0,0,-f],
                    [0,0,1,0]])
    #This transformation matrix is derived from Prof. Didier Stricker's power point presentation on computer vision. 
    #Link : https://ags.cs.uni-kl.de/fileadmin/inf_ags/3dcv-ws14-15/3DCV_lec01_camera.pdf
    Q2 = np.float32([[1,0,0,0],
                    [0,-1,0,0],
                    [0,0,f*0.05,0], #Focal length multiplication obtained experimentally. 
                    [0,0,0,1]])
    threeDPoint = cv2.reprojectImageTo3D(disparity, Q)
    cv2.imshow('3D', threeDPoint)
    '''

cv2.destroyAllWindows()
