import rospy
from geometry_msgs.msg import Twist
import time
import socket
import json
import math as m

def drive_square():
    rospy.init_node('drive_square', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    # Socket setup
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('10.172.200.186', 65433))  # Adjust IP if needed
    server_socket.listen(1)
    conn, addr = server_socket.accept()

    try:
        move_cmd = Twist()
        side_length = 0.5  # Length of each side in meters
        speed = 0.2        # Speed in m/s
        turn_time = 1.57   # Time to turn 90 degrees (in seconds)

        for _ in range(4):
            # Move forward
            move_cmd.linear.x = speed
            move_cmd.angular.z = 0.0
            pub.publish(move_cmd)
            conn.sendall(json.dumps({'linear': speed, 'angular': 0.0}).encode())
            time.sleep(side_length / speed)

            # Stop
            move_cmd.linear.x = 0.0
            pub.publish(move_cmd)
            conn.sendall(json.dumps({'linear': 0.0, 'angular': 0.0}).encode())
            time.sleep(1)

            # Turn 90 degrees
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 1.57 / turn_time  # Adjust angular speed
            pub.publish(move_cmd)
            conn.sendall(json.dumps({'linear': 0.0, 'angular': move_cmd.angular.z}).encode())
            time.sleep(turn_time)

            # Stop turning
            move_cmd.angular.z = 0.0
            pub.publish(move_cmd)
            conn.sendall(json.dumps({'linear': 0.0, 'angular': 0.0}).encode())
            time.sleep(1)

        # Stop the robot at the end
        move_cmd.linear.x = 0.0
        pub.publish(move_cmd)
        conn.sendall(json.dumps({'linear': 0.0, 'angular': 0.0}).encode())

    finally:
        conn.close()
        server_socket.close()

def drive_circle():
    rospy.init_node('drive_circle', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    # Socket setup
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('10.172.200.186', 65433))
    server_socket.listen(1)
    conn, addr = server_socket.accept()

    try:
        move_cmd = Twist()
        radius = 0.01     # Radius of the circle in meters
        speed = -0.2      # Linear speed in m/s
        angular_speed = speed / radius  # Angular velocity to complete the circle

        # Circular motion
        move_cmd.linear.x = speed  # Forward motion
        move_cmd.angular.z = angular_speed  # Rotating while moving forward

        # Move in a circle for a set time or condition
        duration = 10  # Move in a circle for 10 seconds
        start_time = time.time()

        while time.time() - start_time < duration:
            pub.publish(move_cmd)
            conn.sendall(json.dumps({'linear': speed, 'angular': angular_speed}).encode())
            time.sleep(0.1)  # Adjust rate of publishing and sending data

        # Stop the robot after completing the circle
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0
        pub.publish(move_cmd)
        conn.sendall(json.dumps({'linear': 0.0, 'angular': 0.0}).encode())

    finally:
        conn.close()
        server_socket.close()

def turn_degrees(degree):
    rospy.init_node('turn_degrees', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    # Socket setup
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('10.172.200.186', 65433))  
    server_socket.listen(1)
    conn, addr = server_socket.accept()

    try:
        move_cmd = Twist()
        angle = degree * m.pi / 180  # Angle in radiant
        angular_speed = 2  # Angular speed in rad/s
        turn_time = angle / angular_speed

        # Turn the robot by the specified degree
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = angular_speed
        pub.publish(move_cmd)
        conn.sendall(json.dumps({'linear': 0.0, 'angular': move_cmd.angular.z}).encode())
        time.sleep(turn_time)

        # Stop turning
        move_cmd.angular.z = 0.0
        pub.publish(move_cmd)
        conn.sendall(json.dumps({'linear': 0.0, 'angular': 0.0}).encode())
        time.sleep(1)

    finally:
        conn.close()
        server_socket.close()

if __name__ == '__main__':
    try:
        turn_degrees(90)
    except rospy.ROSInterruptException:
        pass


