from serial.tools import list_ports
from serial import Serial
import numpy as np

import time
from enum import Enum, auto
from typing import List
import atexit


class CommsStatus(Enum):
    DISCONNECTED = auto()
    CONNECTED = auto()


class CommsManager:
    _num_ports = 6
    _open_ports: List[Serial] = [None] * _num_ports
    _current_status: CommsStatus = CommsStatus.DISCONNECTED

    @classmethod
    def send_motor_angles(cls, angles: np.ndarray):
        cls.send_array("M", angles)

    @classmethod
    def send_array(cls, prefix: str, array: np.ndarray, suffix = ""):
        # print(array)
        if cls._current_status == CommsStatus.CONNECTED:
            for i, port in enumerate(cls._open_ports):
                # print("Sending to port " + str(i) + ": " + prefix + str(round(array[i], 4)) + suffix + "\n")
                port.write((prefix + str(round(array[i], 4)) + suffix + "\n").encode('UTF-8'))
    
    @classmethod
    def send_message(cls, message):
        # print(message)
        if cls._current_status == CommsStatus.CONNECTED:
            for port in cls._open_ports:
                port.write((str(message) + "\n").encode('UTF-8'))

    @classmethod
    def get_all_replies(cls):
        result: List[str] = []
        if cls._current_status == CommsStatus.CONNECTED:
            for port in cls._open_ports:
                result.append(port.read_all().decode('UTF-8'))
        return result

    @classmethod
    def is_connected(cls):
        return cls._current_status == CommsStatus.CONNECTED

    @classmethod
    def connect(cls):
        if (cls._current_status == CommsStatus.CONNECTED or len(cls._open_ports) > 0):
            for port in cls._open_ports:
                if port is not None:
                    port.close()
            cls._open_ports = [None] * cls._num_ports
            cls._current_status = CommsStatus.DISCONNECTED

        ports = list_ports.comports()

        for port in ports:
            if port.vid == 0x2E8A and port.pid == 0x000A:
                try:
                    temp = Serial(port.name, baudrate=115200)
                    temp.write("CI\n".encode('UTF-8'))
                    time.sleep(0.1)
                    index = int(temp.read_all().decode('UTF-8'))
                    print(index)
                    if (index >= 0 and index < len(cls._open_ports)):
                        cls._open_ports[index] = temp
                    else:
                        print("Error: invalid index " + str(index) + " recieved from port " + temp.name)
                except:
                    pass


        if cls._open_ports.count(None) > 0:
            print("Error: unable to find all ports")
        else:
            cls._current_status = CommsStatus.CONNECTED

    @classmethod
    def disconnect(cls):
        for port in cls._open_ports:
            if port is not None:
                port.close()
        cls._open_ports = [None] * cls._num_ports
        cls._current_status = CommsStatus.DISCONNECTED

    @classmethod
    def go_to_UF2_mode(cls):
        if cls._current_status == CommsStatus.CONNECTED:
            for port in cls._open_ports:
                try:
                    name = port.name
                    port.close()
                    temp = Serial(name, baudrate=1200)
                    temp.close()
                except:
                    pass
            cls._open_ports = [None] * cls._num_ports
            cls._current_status = CommsStatus.DISCONNECTED
        elif cls._current_status == CommsStatus.DISCONNECTED:
            ports = list_ports.comports()
            for port in ports:
                if port.vid == 0x2E8A and port.pid == 0x000A:
                    try:
                        temp = Serial(port.name, baudrate=1200)
                        temp.close()
                    except:
                        pass

    @classmethod
    def clear_prefs(cls):
        if cls._current_status == CommsStatus.CONNECTED:
            for port in cls._open_ports:
                try:
                    print("Now resetting port: " + port.name)
                    port.write(b"CC\n")
                    time.sleep(0.1)
                    print(port.read_all())
                except:
                    pass


atexit.register(CommsManager.disconnect)