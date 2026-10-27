#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, os

if os.name == 'nt':
    import msvcrt
else:
    import tty, termios

# Vận tốc tối đa cho xe tự chế
MAX_LIN_VEL = 0.8
MAX_ANG_VEL = 4.0

LIN_VEL_STEP_SIZE = 0.2
ANG_VEL_STEP_SIZE = 1.0

msg = """
Control Your Mecanum/Omni Drive Robot!
---------------------------
Translation (Linear):
   q    w    e
   a    s    d
   z    x    c

   w/x : Tăng/giảm tốc độ trục X (Tiến / Lùi)
   a/d : Tăng/giảm tốc độ trục Y (Ngang Trái / Ngang Phải)
   q/e : Đi chéo tới trái / chéo tới phải
   z/c : Đi chéo lùi trái / chéo lùi phải

Rotation (Angular):
   j    l
   
   j/l : Tăng/giảm tốc độ trục Z (Xoay Trái / Xoay Phải)

space key, s : Force Stop (Dừng khẩn cấp)

CTRL-C to quit
"""

e = """
Communications Failed
"""

def getKey(settings):
    if os.name == 'nt':
        if sys.version_info[0] >= 3:
            return msvcrt.getch().decode()
        else:
            return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    # Timeout 0.1s - tương đương vòng lặp chạy ở 10Hz
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(target_linear_x, target_linear_y, target_angular_z):
    return "currently:\tLin X: %s\tLin Y: %s\tAng Z: %s " % (
        round(target_linear_x, 3), 
        round(target_linear_y, 3), 
        round(target_angular_z, 3))

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    else:
        output = input
    return output

def constrain(input, low, high):
    if input < low:
        input = low
    elif input > high:
        input = high
    else:
        input = input
    return input

def checkLinearLimitVelocity(vel):
    vel = constrain(vel, -MAX_LIN_VEL, MAX_LIN_VEL)
    return vel

def checkAngularLimitVelocity(vel):
    vel = constrain(vel, -MAX_ANG_VEL, MAX_ANG_VEL)
    return vel

def main(args=None):
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    # Khởi tạo rclpy
    rclpy.init(args=args)
    
    # Tạo Node và Publisher kiểu ROS 2
    node = rclpy.create_node('mecanum_teleop')
    pub = node.create_publisher(Twist, 'cmd_vel', 10)

    status = 0
    
    # Target velocities
    target_linear_x_vel   = 0.0
    target_linear_y_vel   = 0.0
    target_angular_z_vel  = 0.0
    
    # Control velocities (sau khi làm mượt)
    control_linear_x_vel  = 0.0
    control_linear_y_vel  = 0.0
    control_angular_z_vel = 0.0

    try:
        print(msg)
        while True:
            key = getKey(settings)
            
            # Xử lý tịnh tiến (Tiến/Lùi)
            if key == 'w':
                target_linear_x_vel = checkLinearLimitVelocity(target_linear_x_vel + LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_x_vel, target_linear_y_vel, target_angular_z_vel))
            elif key == 'x':
                target_linear_x_vel = checkLinearLimitVelocity(target_linear_x_vel - LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_x_vel, target_linear_y_vel, target_angular_z_vel))
                
            # Xử lý tịnh tiến ngang (Trái/Phải)
            elif key == 'a':
                target_linear_y_vel = checkLinearLimitVelocity(target_linear_y_vel + LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_x_vel, target_linear_y_vel, target_angular_z_vel))
            elif key == 'd':
                target_linear_y_vel = checkLinearLimitVelocity(target_linear_y_vel - LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_x_vel, target_linear_y_vel, target_angular_z_vel))
                
            # Xử lý đi chéo (Tới-Trái / Tới-Phải)
            elif key == 'q':
                target_linear_x_vel = checkLinearLimitVelocity(target_linear_x_vel + LIN_VEL_STEP_SIZE)
                target_linear_y_vel = checkLinearLimitVelocity(target_linear_y_vel + LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_x_vel, target_linear_y_vel, target_angular_z_vel))
            elif key == 'e':
                target_linear_x_vel = checkLinearLimitVelocity(target_linear_x_vel + LIN_VEL_STEP_SIZE)
                target_linear_y_vel = checkLinearLimitVelocity(target_linear_y_vel - LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_x_vel, target_linear_y_vel, target_angular_z_vel))

            # Xử lý đi chéo (Lùi-Trái / Lùi-Phải)
            elif key == 'z':
                target_linear_x_vel = checkLinearLimitVelocity(target_linear_x_vel - LIN_VEL_STEP_SIZE)
                target_linear_y_vel = checkLinearLimitVelocity(target_linear_y_vel + LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_x_vel, target_linear_y_vel, target_angular_z_vel))
            elif key == 'c':
                target_linear_x_vel = checkLinearLimitVelocity(target_linear_x_vel - LIN_VEL_STEP_SIZE)
                target_linear_y_vel = checkLinearLimitVelocity(target_linear_y_vel - LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_x_vel, target_linear_y_vel, target_angular_z_vel))
                
            # Xử lý xoay tại chỗ (Rotation)
            elif key == 'j':
                target_angular_z_vel = checkAngularLimitVelocity(target_angular_z_vel + ANG_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_x_vel, target_linear_y_vel, target_angular_z_vel))
            elif key == 'l':
                target_angular_z_vel = checkAngularLimitVelocity(target_angular_z_vel - ANG_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_x_vel, target_linear_y_vel, target_angular_z_vel))
                
            # Dừng khẩn cấp
            elif key == ' ' or key == 's':
                target_linear_x_vel   = 0.0
                target_linear_y_vel   = 0.0
                target_angular_z_vel  = 0.0
                
                control_linear_x_vel  = 0.0
                control_linear_y_vel  = 0.0
                control_angular_z_vel = 0.0
                print(vels(target_linear_x_vel, target_linear_y_vel, target_angular_z_vel))
                
            else:
                if (key == '\x03'): # Bắt sự kiện CTRL-C
                    break

            if status == 20:
                print(msg)
                status = 0

            twist = Twist()

            # Nội suy làm mượt vận tốc (acceleration profile) cho cả 3 trục
            control_linear_x_vel = makeSimpleProfile(control_linear_x_vel, target_linear_x_vel, (LIN_VEL_STEP_SIZE/2.0))
            twist.linear.x = float(control_linear_x_vel)
            
            control_linear_y_vel = makeSimpleProfile(control_linear_y_vel, target_linear_y_vel, (LIN_VEL_STEP_SIZE/2.0))
            twist.linear.y = float(control_linear_y_vel)
            
            twist.linear.z = 0.0

            control_angular_z_vel = makeSimpleProfile(control_angular_z_vel, target_angular_z_vel, (ANG_VEL_STEP_SIZE/2.0))
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = float(control_angular_z_vel)

            # Publish liên tục ở ~10Hz
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        # Hãm phanh toàn bộ các trục trước khi tắt node
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            
        # Dọn dẹp memory của ROS 2
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()