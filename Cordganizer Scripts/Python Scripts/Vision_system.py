import rtde_control
import rtde_receive
import rtde_io
import time
import cv2
import numpy as np

z = 0.4

# Connect to robot
rtde_c = rtde_control.RTDEControlInterface("10.241.34.45")
rtde_r = rtde_receive.RTDEReceiveInterface("10.241.34.45")
rtde_io_ = rtde_io.RTDEIOInterface("10.241.34.45")

# Define poses
p_home = [0.10914, -0.48692, 0.03133+z, 0.0, 3.142, 0.0]
p_home_to_pallet =  [0.3219,0.38565,-0.20525+z,3.126,0.008,-0.008]
p_home_to_pallet_down =  [0.3219,0.38565,-0.25174+z,3.126,0.008,-0.008]
pallet_to_cam = [0.487,-0.0833,-0.07655+z,2.228,-2.203,-0.021]
p_home_cam = [0.8007,-0.0669,-0.355+z,2.198,-2.258,-0.005]
p_home_abvcam = [0.8007,-0.0669,-0.285+z,2.198,-2.258,-0.005]




away_from_cam =[0.59510,-0.09503,-0.19215+z,2.193,-2.145,-0.046]

pick_up =[0.8007,-0.0669,-0.3658+z,2.198,-2.258,-0.005]

white_position =[0.18706,-0.51690,-0.2807+z,2.249,2.204,0.037]

blue_position =[0.29910,-0.49650,-0.2807+z,2.242,2.148,-0.035]

def classify_color(bgr):
    b, g, r = bgr
    if b < 180 and g < 120 and r < 130:
        return "Blue"
    elif r > 120 and g > 120 and b > 120:
        return "White"
    else:
        return "Unknown"

def get_object_color():
    cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
    x1, y1 = 140, 50
    x2, y2 = 450, 360

    for _ in range(30):  # Let camera adjust
        ret, frame = cap.read()

    ret, frame = cap.read()
    cap.release()

    if not ret:
        return "Unknown"

    frame = cv2.resize(frame, (640, 480))
    roi = frame[y1:y2, x1:x2]

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 30, 100)

    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 1000:
            x, y, w, h = cv2.boundingRect(cnt)
            obj_roi = roi[y:y+h, x:x+w]
            b, g, r = cv2.mean(obj_roi)[:3]
            return classify_color((b, g, r))

    return "Unknown"

rtde_io_.setStandardDigitalOut(0, True)
print("Gripper closed")
time.sleep(1)

# Move to p_home
q_home = rtde_c.getInverseKinematics(p_home)
rtde_c.moveJ(q_home, speed=0.3)
time.sleep(1)


q_next = rtde_c.getInverseKinematics(p_home_to_pallet)
rtde_c.moveJ(q_next, speed=0.3)
time.sleep(1)



rtde_io_.setStandardDigitalOut(0, False)
print("Gripper opened")
time.sleep(1)


q_next = rtde_c.getInverseKinematics(p_home_to_pallet_down)
rtde_c.moveJ(q_next, speed=0.3)
time.sleep(1)

rtde_io_.setStandardDigitalOut(0, True)
print("Gripper closed")
time.sleep(1)

q_next = rtde_c.getInverseKinematics(p_home_to_pallet)
rtde_c.moveJ(q_next, speed=0.3)
time.sleep(1)

q_next = rtde_c.getInverseKinematics(pallet_to_cam)
rtde_c.moveJ(q_next, speed=0.3)
time.sleep(1)




q_home = rtde_c.getInverseKinematics(p_home_cam)
rtde_c.moveJ(q_home, speed=0.3)
time.sleep(1)

rtde_io_.setStandardDigitalOut(0, False)
print("Gripper opened")
time.sleep(1)

q_home = rtde_c.getInverseKinematics(p_home_abvcam)
rtde_c.moveJ(q_home, speed=0.3)
time.sleep(1)



q_home = rtde_c.getInverseKinematics(away_from_cam)
rtde_c.moveJ(q_home, speed=0.3)
time.sleep(1)

color = get_object_color()
print(f"Detected color: {color}")

q_home = rtde_c.getInverseKinematics(p_home_abvcam)
rtde_c.moveJ(q_home, speed=0.3)
time.sleep(1)

q_home = rtde_c.getInverseKinematics(pick_up)
rtde_c.moveJ(q_home, speed=0.3)
time.sleep(1)

rtde_io_.setStandardDigitalOut(0, True)
print("Gripper closed")
time.sleep(1)

q_home = rtde_c.getInverseKinematics(p_home)
rtde_c.moveJ(q_home, speed=0.3)
time.sleep(1)

if color == "White":
    q_home = rtde_c.getInverseKinematics(white_position)
    rtde_c.moveJ(q_home, speed=0.3)
    time.sleep(1)
    rtde_io_.setStandardDigitalOut(0, False)
    print("Gripper opened")
    time.sleep(1)
elif color == "Blue":
    q_home = rtde_c.getInverseKinematics(blue_position)
    rtde_c.moveJ(q_home, speed=0.3)
    time.sleep(1)
    rtde_io_.setStandardDigitalOut(0, False)
    print("Gripper opened")
    time.sleep(1)
else:
    print("Unrecognized color.")

q_home = rtde_c.getInverseKinematics(p_home)
rtde_c.moveJ(q_home, speed=0.3)
time.sleep(1)


rtde_io_.setStandardDigitalOut(0, True)
print("Gripper closed")
time.sleep(1)






# Stop control script
rtde_c.stopScript()
