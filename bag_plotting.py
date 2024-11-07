import bagpy
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class OdomPlotting:
    def __init__(self, bag_name):
        # Store the bag name and load the data
        self.bag_name = bag_name
        self.odom_df = pd.read_csv(f"{bag_name}/odom.csv")

        # Set up the figure and axis
        self.fig, self.ax = plt.subplots()
        (self.line1,) = self.ax.plot([], [], lw=1, label="Angular Z")
        (self.line2,) = self.ax.plot([], [], lw=1, label="Linear X")

        # Set labels and legend
        self.ax.set_title("Live Plot of Angular Z and Linear X Over Time")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Value (rad/s for Angular Z, m/s for Linear X)")

        # Set legend depending on the bag name
        if "kreis" in bag_name:
            self.ax.legend(loc="upper left")
        else:
            self.ax.legend(loc="lower left")

        # Create the animation
        self.ani = FuncAnimation(
            self.fig,
            self.update,
            frames=range(0, len(self.odom_df), 5),
            init_func=self.init,
            blit=True,
            interval=10,
            save_count=len(self.odom_df),
        )

    # Initialization function for the plot
    def init(self):
        self.ax.set_xlim(self.odom_df["Time"].min(), self.odom_df["Time"].max())
        # Set y-limits to fit both data ranges
        min_y = min(self.odom_df["twist.twist.angular.z"].min(), self.odom_df["twist.twist.linear.x"].min()) - 0.1
        max_y = max(self.odom_df["twist.twist.angular.z"].max(), self.odom_df["twist.twist.linear.x"].max()) + 0.1
        self.ax.set_ylim(min_y, max_y)
        return self.line1, self.line2

    # Update function for animation
    def update(self, frame):
        x_data = self.odom_df["Time"][:frame]
        y_data_angular_z = self.odom_df["twist.twist.angular.z"][:frame]
        y_data_linear_x = self.odom_df["twist.twist.linear.x"][:frame]
        self.line1.set_data(x_data, y_data_angular_z)
        self.line2.set_data(x_data, y_data_linear_x)
        return self.line1, self.line2

    # Function to display the plot
    def show(self):
        plt.show()


class AccPlotting:
    def __init__(self, bag_name):
        self.imu_df = pd.read_csv(f"{bag_name}/imu.csv")

        # Set up the figure and axes for acceleration
        self.fig, self.ax = plt.subplots()
        (self.line1,) = self.ax.plot([], [], lw=1, label="lin. accel. X")
        (self.line2,) = self.ax.plot([], [], lw=1, label="lin. accel. Y")
        (self.line3,) = self.ax.plot([], [], lw=1, label="lin. accel. Z")

        # Set labels and legend
        self.ax.set_title("Linear Acceleration along X, Y, and Z over Time")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Acceleration (m/s^2)")
        self.ax.legend(loc="upper left")

        # Create the animation
        self.ani = FuncAnimation(
            self.fig,
            self.update,
            frames=range(0, len(self.imu_df), 5),
            init_func=self.init,
            blit=True,
            interval=10,
            save_count=len(self.imu_df),
        )

    # Initialization function for the acceleration plot
    def init(self):
        self.ax.set_xlim(self.imu_df["Time"].min(), self.imu_df["Time"].max())
        min_y = min(self.imu_df["linear_acceleration.x"].min(), 
                    self.imu_df["linear_acceleration.y"].min(), 
                    self.imu_df["linear_acceleration.z"].min()) - 0.1
        max_y = max(self.imu_df["linear_acceleration.x"].max(), 
                    self.imu_df["linear_acceleration.y"].max(), 
                    self.imu_df["linear_acceleration.z"].max()) + 0.1
        self.ax.set_ylim(min_y, max_y)
        return self.line1, self.line2, self.line3

    # Update function for acceleration animation
    def update(self, frame):
        x_data = self.imu_df["Time"][:frame]
        y_data_accel_x = self.imu_df["linear_acceleration.x"][:frame]
        y_data_accel_y = self.imu_df["linear_acceleration.y"][:frame]
        y_data_accel_z = self.imu_df["linear_acceleration.z"][:frame]
        self.line1.set_data(x_data, y_data_accel_x)
        self.line2.set_data(x_data, y_data_accel_y)
        self.line3.set_data(x_data, y_data_accel_z)
        return self.line1, self.line2, self.line3

    # Function to display the plot
    def show(self):
        plt.show()


class GyroPlotting:
    def __init__(self, bag_name):
        self.imu_df = pd.read_csv(f"{bag_name}/imu.csv")

        # Set up the figure and axes for gyroscope
        self.fig, self.ax = plt.subplots()
        (self.line1,) = self.ax.plot([], [], lw=1, label="ang. vel. X")
        (self.line2,) = self.ax.plot([], [], lw=1, label="ang. vel. Y")
        (self.line3,) = self.ax.plot([], [], lw=1, label="ang. vel. Z")

        # Set labels and legend
        self.ax.set_title("Angular Velocity around X, Y, and Z over Time")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Angular Velocity (rad/s)")
        self.ax.legend(loc="upper left")

        # Create the animation
        self.ani = FuncAnimation(
            self.fig,
            self.update,
            frames=range(0, len(self.imu_df), 5),
            init_func=self.init,
            blit=True,
            interval=10,
            save_count=len(self.imu_df),
        )

    # Initialization function for the gyroscope plot
    def init(self):
        self.ax.set_xlim(self.imu_df["Time"].min(), self.imu_df["Time"].max())
        min_y = min(self.imu_df["angular_velocity.x"].min(), 
                    self.imu_df["angular_velocity.y"].min(), 
                    self.imu_df["angular_velocity.z"].min()) - 0.1
        max_y = max(self.imu_df["angular_velocity.x"].max(), 
                    self.imu_df["angular_velocity.y"].max(), 
                    self.imu_df["angular_velocity.z"].max()) + 0.1
        self.ax.set_ylim(min_y, max_y)
        return self.line1, self.line2, self.line3

    # Update function for gyroscope animation
    def update(self, frame):
        x_data = self.imu_df["Time"][:frame]
        y_data_gyro_x = self.imu_df["angular_velocity.x"][:frame]
        y_data_gyro_y = self.imu_df["angular_velocity.y"][:frame]
        y_data_gyro_z = self.imu_df["angular_velocity.z"][:frame]
        self.line1.set_data(x_data, y_data_gyro_x)
        self.line2.set_data(x_data, y_data_gyro_y)
        self.line3.set_data(x_data, y_data_gyro_z)
        return self.line1, self.line2, self.line3

    # Function to display the plot
    def show(self):
        plt.show()


# Run the code if this file is executed directly
if __name__ == "__main__":
    # # Create the OdomPlotting object
    # odom_plot = OdomPlotting("411_bewegt")
    # odom_plot.show()

    # # Create the AccPlotting object
    # acc_plot = AccPlotting("411_bewegt")
    # acc_plot.show()

    # Create the GyroPlotting object
    gyro_plot = GyroPlotting("411_bewegt")
    gyro_plot.show()


    pass

