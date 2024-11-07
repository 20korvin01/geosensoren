
from datetime import datetime
import serial
import time
import numpy as np
import random



def request(port: str, command: str) -> bytes:
    """
    Sends a command to totalstation and returns response.
    """
    with serial.Serial(port) as s:
        s.baudrate = 115200
        s.bytesite = serial.EIGHTBITS
        s.parity = serial.PARITY_NONE
        s.timeout = None

        command = f"\n{command}\r\n"
        s.write(command.encode())
        time.sleep(0.01)
        response = s.readline()
    return response

def set_datetime(port: str) -> None:
    """
    Sets the date of totalstation to current date.
    """
    date = datetime.now()
    command = f"%R1Q,5007:{date.year},{hex(date.month)[2:].zfill(2)},{hex(date.day)[2:].zfill(2)},{hex(date.hour)[2:].zfill(2)},{hex(date.minute)[2:].zfill(2)},{hex(date.second)[2:].zfill(2)}"
    request(port, command)

def get_face(port: str) -> int:
    """
    Checks which face of totalstation is currently in use.
    """
    command = "%R1Q,2026:"
    face = request(port, command).decode("utf-8").strip("\r\n").split(",")[3]
    return int(face)

def set_ATR(port: str, io: int) -> bytes:
    command = f"%R1Q,18005:{io}"
    return request(port, command)


def switch_face(port: str) -> None:
    """
    Switches face of totalstation to other side.
    """
    command = "%R1Q,9028:1,1,0"
    return request(port, command)

def move_to_target(port: str, Hz: float, V: float) -> None:
    """
    Moves totalstation to given horizontal and vertical angles.
    """
    command = f"%R1Q,9027:{Hz},{V},1,1,0"
    request(port, command)

def measure_distance(port: str) -> None:
    """
    Measures distance to target.
    """
    command = "%R1Q,2008:1,1"
    request(port, command)

def get_polar(port: str, delay: int) -> list:
    """
    Returns polar coordinates of target.
    """
    command = f"%R1Q,2167:{delay},1"
    response = request(port, command).decode("utf-8").strip("\r\n").split(",")
    return response

def get_cartesian(port: str, delay: int) -> list:
    """
    Returns cartesian coordinates of target.
    """
    command = f"%R1Q,2116:{delay},1"
    response = request(port, command).decode("utf-8").strip("\r\n").split(",")
    return response

def laserpointer(port: str, io: bool) -> None:
    """
    Turns laserpointer on or off.
    """
    if io:
        command = "%R1Q,1004:1"
    else:
        command = "%R1Q,1004:0"
    request(port, command)

def stop(port: str) -> None:
    """
    Stops all movements of totalstation.
    """
    command = "%R1Q,6002:1"
    request(port, command)

def set_orientation(port: str) -> None:
    """
    Sets orientation of totalstation to 0.
    """
    command = "%R1Q,2113:0"
    request(port, command)

def search_target(port: str, Hz: float, V: float) -> bytes:
    """
    Searches for target at given horizontal and vertical angles.
    """
    command = f"%R1Q,9029:{Hz},{V},0"
    return request(port, command)

def set_position(port: str, x, y, z) -> None:
    """
    Sets position of totalstation to given cartesian coordinates.
    """
    command = f"%R1Q,2010:{x},{y},{z},0"
    request(port, command)

def speed(port: str, Hz: float, V: float) -> None:
    """
    Sets horizontal and vertical speed of totalstation.
    """
    command = f"%R1Q,6004:{Hz},{V}"
    return request(port, command)

def start_ts(port: str) -> None:
    """
    Starts totalstation for movings.
    """
    command = "%R1Q,6001:1"
    request(port, command)


def find_start(port: str, point_file_path: str) -> None:
    # set position of totalstation
    set_position(port, 1000, 1000, 10)
    # data
    data = np.genfromtxt(f"Data/{point_file_path}", delimiter=",", skip_header=1)
    targets = data[:, 0]
    distances = data[:, 3]
    Hzs = data[:, 1]
    Vs = data[:, 2]
    # look straight
    Hz = get_polar(port, 0)[3]
    move_to_target(port, Hz, 0.5*np.pi)
    # start scanning
    stop = False
    while not stop:
        # search for target horizontally
        search_target(port, np.pi, 0)
        measure_distance(port)
        # get polar coordinates
        delay = 3000
        response = get_polar(port, delay)
        polar = [response[i] for i in [3, 4, 9]]
        # check if target is in range
        for distance in distances:
            if abs(float(polar[2])-distance) < 0.01:
                stop = True
                break
        if stop:
            # get index of distance in distances
            idx = np.where(distances == distance)
            # get target number
            target = targets[idx]
        else:
            # start totalstation
            start_ts(port)
            # move horizontally for 0.1 seconds
            speed(port, -0.5, 0)
            time.sleep(0.5)
            speed(port, 0, 0)
    # get Hz and V from found target
    target_Hz = Hzs[idx][0]
    instr_Hz  = float(get_polar(port, 0)[3])
    target_V  = Vs[idx][0]
    # move to target 1001
    move_to_target(port, instr_Hz-target_Hz, target_V)
    # set orientation
    set_orientation(port)

def run(port: str, point_file_path: str, result_file_path: str, wait_time: int, num_ep: int) -> None:
    targets = np.genfromtxt(f"Data/{point_file_path}", delimiter=",", skip_header=1)

    # check face and switch if necessary
    face = get_face(port)
    if face == "1": # face 2 detected
        # switch to face 1
        switch_face(port)

    for i in range(num_ep):
        for target in targets:
            # move to target and do fine adjustment
            Hz = target[1]
            V  = target[2]
            move_to_target(port, Hz, V)
            # target number
            nr = target[0]
            # timestamp
            timestamp = datetime.now().strftime("%Y.%m.%d-%H:%M:%S")
            # measure distance
            measure_distance(port)
            # get polar coordinates
            delay = 3000
            response = get_polar(port, delay)
            polar = [response[i] for i in [3, 4, 9]]
            # get cartesian coordinates
            delay = 0
            response = get_cartesian(port, delay)
            cartesian = [response[i] for i in [3, 4, 5]]
            # save data
            data = [int(nr)] + cartesian + polar  + [1] + [timestamp]
            with open(f"Data/{result_file_path}", "a") as f:
                f.write(",".join([str(d) for d in data]) + "\n")

        # switch to face 2
        switch_face(port)

        for target in reversed(targets):
            # move to target and do fine adjustment
            Hz = target[1] + np.pi
            V  = target[2] + np.pi
            move_to_target(port, Hz, V)
            # target number
            nr = target[0]
            # timestamp
            timestamp = datetime.now().strftime("%Y.%m.%d-%H:%M:%S")
            # measure distance
            measure_distance(port)
            # get polar coordinates
            delay = 3000
            response = get_polar(port, delay)
            polar = [response[i] for i in [3, 4, 9]]
            # get cartesian coordinates
            delay = 0
            response = get_cartesian(port, delay)
            cartesian = [response[i] for i in [3, 4, 5]]
            # save data
            data = [int(nr)] + cartesian + polar  + [2] + [timestamp]
            with open(f"Data/{result_file_path}", "a") as f:
                f.write(",".join([str(d) for d in data]) + "\n")

        # switch to face 1
        switch_face(port)

        # wait if loop is not finished
        if i < num_ep - 1:
            time.sleep(wait_time)


def fun1(port: str) -> None:
    # start totalstation
    start_ts(port)
    # slowly increase the horizontal velocity and the vertical velocity
    for i in range(0, 100, 10):
        speed(port, i/100, i/100)
        time.sleep(0.001)

    # slowly decrease the horizontal velocity and the vertical velocity
    for i in range(0, 100, 10):
        speed(port, (100-i)/100, (100-i)/100)
        time.sleep(0.001)

    # set it to zero
    s = speed(port, 0, 0)

def fun2(port: str) -> None:
    # start totalstation
    start_ts(port)
    # slowly increase the vertical velocity
    for i in range(0, 100, 10):
        speed(port, 0, i/100)
        time.sleep(0.001)

    # slowly decrease the vertical velocity
    for i in range(0, 100, 10):
        speed(port, 0, (100-i)/100)
        time.sleep(0.001)

    # set it to zero
    s = speed(port, 0, 0)

def fun3(port: str) -> None:
    # start totalstation
    start_ts(port)
    # slowly increase the horizontal velocity
    for i in range(0, 100, 10):
        speed(port, i/100, 0)
        time.sleep(0.001)

    # slowly decrease the horizontal velocity
    for i in range(0, 100, 10):
        speed(port, (100-i)/100, 0)
        time.sleep(0.001)

    # set it to zero
    s = speed(port, 0, 0)

def fun4(port: str) -> None:
    command = "%R1Q,11004:"
    request(port, command)

def fun(port: str) -> None:
    functions = [fun1, fun2, fun3, fun4]
    selected_function = random.choice(functions)
    selected_function(port)


if __name__ == "__main__":
    port = "COM6"

    set_ATR(port, 1)
    find_start(port, "Epoche0_1001-4.csv")
    # run(port, "Epoche0_1001-3.csv", "measurements.csv", 30, 2)
    # print(speed(port, 0.5, 0.5))
    # time.sleep(2)
    # speed(port, 0, 0)
    # print(switch_face(port))
    # pass