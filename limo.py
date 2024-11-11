import paramiko
import time

def command(cmd):
    full_cmd = f"source agilex_ws/devel/setup.bash && {cmd}"
    print(full_cmd)
    _stdin, _stdout, _stderr = client.exec_command(full_cmd)
    if _stderr.channel.recv_exit_status() != 0:
        msg = _stderr.read().decode()
    else:
        msg = _stdout.read().decode()
    return msg

if __name__ == "__main__":
    host = "10.172.200.186"
    username = "agilex"
    password = "agx"

    client = paramiko.client.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(host, username=username, password=password)

    start = time.time()
    # driving the robot forward
    first = command("rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[1.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]' &")
    print(first)
    end = time.time()
    print(f"Elapsed time: {end-start}")
    start = end

    # driving the robot backward
    result = command("rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[-1.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'")
    print(result)
    end = time.time()
    print(f"Elapsed time: {end-start}")


    client.close()
