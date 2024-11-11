import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class DriveStraight:

    def __init__(self, target_distance, target_speed):
        rospy.init_node('drive_straight', anonymous=True)
        self.publisher_ = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.loginfo('Drive Straight Node has been started')

        self.target_distance = target_distance  # Target distance to travel
        self.target_speed = target_speed  # Target speed set by user
        self.distance_travelled = 0.0  # To accumulate the total distance travelled
        self.previous_time = 0  # To store the timestamp of the previous odom message

        # Subscriber to listen to the odometry topic
        self.subscriber_ = rospy.Subscriber('/odom', Odometry, self.odometry_callback)

    def odometry_callback(self, msg):
        # TIME
        current_time = msg.header.stamp.to_sec()
        delta_time = current_time - self.previous_time
        print(delta_time)
        self.previous_time = current_time
        # VELOCITY
        current_velocity = msg.twist.twist.linear.x
        # print(current_velocity)
        # DISTANCE
        distance = current_velocity * delta_time
        self.distance_travelled += distance
        # print(self.distance_travelled)
        # DRIVE CONTROL
        # Check if the target distance has been reached
        if self.distance_travelled > self.target_distance:
            msg = Twist()
            msg.linear.x = 0  # Stop the robot
            self.publisher_.publish(msg)
            # rospy.loginfo(f"Target distance of {self.target_distance} meters reached. Stopping robot.")
        else:
            # msg = Twist()
            # msg.linear.x = self.target_speed  # Set the target speed
            # self.publisher_.publish(msg)
            # # rospy.loginfo(f"Distance travelled so far: {self.distance_travelled} meters")
            msg = Twist()
            msg.linear.x = 0  # Stop the robot
            self.publisher_.publish(msg)
def main():
    # Set the desired target speed and target distance
    target_speed = 0.1  # m/s
    target_distance = 1.0  # m

    # Start driving
    drive_straight = DriveStraight(target_distance, target_speed)
    rospy.spin()

if __name__ == '__main__':
    main()
