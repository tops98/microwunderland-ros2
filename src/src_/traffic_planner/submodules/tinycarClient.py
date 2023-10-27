import os
import math
import socket
from enum import Enum
from threading import Lock
from typing import Tuple


MSG_TYPE_LIGHTS = 0x01
MSG_TYPE_MOTOR = 0x02
MSG_TYPE_BATTERY_STATUS_REQ = b'\x05'
BATTERY_STATUS = 0x03


class BlinkerStates (Enum):
    ON = 0
    OFF = 1
    HAZARD = 2
class TaillightStates (Enum):
    ON = 0
    OFF = 1
    BRAKE = 2
class HeadlightStates (Enum):
    ON = 0
    OFF = 1


class TinycarClient:
    
    net_lock = Lock()   # static member

    _receive_socket: socket.socket
    _send_socket: socket.socket
    _tinycar_address: Tuple[str,int]
    _headlightState:BlinkerStates
    _taillightState:BlinkerStates
    _blinkerState:BlinkerStates

    def __init__(self, hostname:str, tinycar_port:int, local_port:int) -> None:
        self._send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._receive_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._receive_socket.bind("0.0.0.0",local_port)
        self._tinycar_address = (hostname, tinycar_port)

        _headlightState = HeadlightStates.OFF
        _taillightState = TaillightStates.OFF
        _blinkerState = BlinkerStates.OFF


    def setMotorSpeed(self, speed:int) ->None:
        data = bytearray([MSG_TYPE_MOTOR,speed])
        self._sendData(bytes(data))
    
    
    def setHeadlightState(self, state:HeadlightStates) -> None:
        self._headlightState = state
        self._setLights()
    
    def setTaillightState(self, state:TaillightStates) -> None:
        self._taillightState = state
        self._setLights()
    
    def setBlinkertState(self, state:BlinkerStates) -> None:
        self._blinkerState = state
        self._setLights()
    
    def _setLights(self) -> None:
        data = bytearray([
            MSG_TYPE_LIGHTS,
            self._headlightState.value,
            self._taillightState.value,
            self._blinkerState.value
        ])
        self._sendData(bytes(data))
    
    def get_battery_status(self) -> float:
        data = self._send_and_receive(data=MSG_TYPE_BATTERY_STATUS_REQ)
        if data[0] == BATTERY_STATUS:
            voltage = int.from_bytes(data[1:3], byteorder='little') / 1000 # from mV to V
            percent = self._voltage_to_percent(voltage)
            
            return percent
        else:
            raise ValueError(f"unknown response for battery status request: {data[0]}")

    
    def _send_and_receive(self, data:bytes) -> bytes:
        with TinycarClient.net_lock:
            self._sendData(data)
            return self._receiveData
        

    def _sendData(self,data:bytes) -> None:
        with self._send_socket as s:
            s.sendto(data, self._tinycar_address)


    def _receiveData(self, buffsize:int = 1024) -> bytes:
        with self._receive_socket as s:
            data, addr = s.recvfrom(buffsize)
            return data


    def _voltage_to_percent(self, voltage: float) ->float:
            # LiPo has 3.7V nominal voltage, 4.2V max voltage and 3.0V min voltage (better not to go below 3.3V)
            # 3.3V = 0%
            # 4.2V = 100%
            # 3.85V = 50%
            # use non linear function
            return (100 * (math.exp(voltage - 3.3) - 1) / (math.exp(4.2 - 3.3) - 1))
    
    

