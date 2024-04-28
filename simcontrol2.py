import socket
import struct
import sys

buffer_size = 4096

def uint_to_bytes(val : int) -> bytes:
    return int.to_bytes(val, 4, byteorder=sys.byteorder, signed=False)

def bytes_to_uint(data : bytes, offset : int) -> int:
    return int.from_bytes(data[offset:offset+4], byteorder=sys.byteorder, signed=False)

class MessagingClient:
    def __init__(self, host : str, port : int):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((host, port))

    def request(self, data : bytes) -> bytes:
        self.socket.send(data)
        return self.socket.recv(buffer_size)
    
    def close(self):
        self.socket.close()

class Controller:
    def __init__(self, host : str, port : int):
        self.messaging = MessagingClient(host, port)

    # Parameters: none
    # Returns: time step
    def get_time_step(self) -> float:
        reply = self.messaging.request(uint_to_bytes(0))
        return struct.unpack('d', reply)[0]

    # Parameters: actuator name
    # Returns: None or (actuator index, actuator dof)
    def get_actuator_info(self, name : str) -> tuple[int, int]:
        request = uint_to_bytes(1)
        array = name.encode("utf-8")
        request += int.to_bytes(len(array), length=4, byteorder=sys.byteorder, signed=False)
        request += array
        reply = self.messaging.request(request)
        if reply[0:1] != b'\x00':
            return None
        else:
            index = int.from_bytes(reply[1:5], byteorder=sys.byteorder, signed=False)
            dof = int.from_bytes(reply[5:9], byteorder=sys.byteorder, signed=False)
            return index, dof
        
    # Parameters: sensor name
    # Returns: None or (sensor index, sensor dof)
    def get_sensor_info(self, name : str) -> tuple[int, int]:
        request = uint_to_bytes(2)
        array = name.encode("utf-8")
        request += int.to_bytes(len(array), length=4, byteorder=sys.byteorder, signed=False)
        request += array
        reply = self.messaging.request(request)
        if reply[0:1] != b'\x00':
            return None
        else:
            index = int.from_bytes(reply[1:5], byteorder=sys.byteorder, signed=False)
            dof = int.from_bytes(reply[5:9], byteorder=sys.byteorder, signed=False)
            return index, dof
        
    # Parameters: none
    # Returns: (success, current state), where state = 0 for not running, 1 for running.
    def start(self) -> tuple[bool, int]:
        request = uint_to_bytes(3)
        request += b'\x00'
        reply = self.messaging.request(request)
        success = reply[0:1] == b'\x00'
        state = int.from_bytes(reply[1:2], byteorder=sys.byteorder, signed=False)
        return success, state
    
    # Parameters: none
    # Returns: (success, current state), where state = 0 for not running, 1 for running.
    def reset(self) -> tuple[bool, int]:
        request = uint_to_bytes(3)
        request += b'\x01'
        reply = self.messaging.request(request)
        success = reply[0:1] == b'\x00'
        state = int.from_bytes(reply[1:2], byteorder=sys.byteorder, signed=False)
        return success, state
    
    # Parameters: none
    # Returns: (success, current state), where state = 0 for not running, 1 for running.
    def clear(self) -> tuple[bool, int]:
        request = uint_to_bytes(3)
        request += b'\x02'
        reply = self.messaging.request(request)
        success = reply[0:1] == b'\x00'
        state = int.from_bytes(reply[1:2], byteorder=sys.byteorder, signed=False)
        return success, state
    
    # Parameters: steps to simulate, actuator inputs
    # (actuator inputs: index (int) -> input (tuple of float))
    # Returns: (status code, sensor outputs), where status code = 0 for success, 1 for inpur error, 2 for simulation error. 
    # (sensor outputs: index (int) -> output (tuple of float))
    def simulate(self, steps : int, actuator_inputs : dict) -> tuple[int, dict]:
        request = uint_to_bytes(4)
        request += int.to_bytes(steps, length=4, byteorder=sys.byteorder, signed=False)
        request += int.to_bytes(len(actuator_inputs), length=4, byteorder=sys.byteorder, signed=False)
        for index in actuator_inputs:
            request += int.to_bytes(index, length=4, byteorder=sys.byteorder, signed=False)
            input = actuator_inputs[index]
            request += int.to_bytes(len(input), length=4, byteorder=sys.byteorder, signed=False)
            for val in input:
                request += struct.pack('d', val)

        reply = self.messaging.request(request)
        status_code = int.from_bytes(reply[0:1], byteorder=sys.byteorder, signed=False)
        if status_code != 0:
            return (status_code, None)
        else:
            sensor_count = int.from_bytes(reply[1:5], byteorder=sys.byteorder, signed=False)
            sensor_outputs = {}
            start = 5
            for i in range(sensor_count):
                index = int.from_bytes(reply[start:start+4], byteorder=sys.byteorder, signed=False)
                start += 4
                dof = int.from_bytes(reply[start:start+4], byteorder=sys.byteorder, signed=False)
                start += 4
                output = []
                for j in range(dof):
                    output.append(struct.unpack('d', reply[start:start+8])[0])
                    start += 8
                sensor_outputs[index] = tuple(output)
            
            return (status_code, sensor_outputs)
        
    # Parameters: sequential number
    # Returns: success or not
    def export(self, seq_num : int) -> bool:
        request = uint_to_bytes(5)
        request += uint_to_bytes(seq_num)
        reply = self.messaging.request(request)
        return reply[0:1] == b'\x00'

    def close(self):
        self.messaging.close()