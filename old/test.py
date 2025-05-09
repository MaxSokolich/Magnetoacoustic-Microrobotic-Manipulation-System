import serial.tools.list_ports

def get_arduino_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if "Arduino" in port.description or "CH340" in port.description or "ttyUSB" in port.device:
            return port.device  # This is the serial port string like 'COM3' or '/dev/ttyUSB0'
    return None

arduino_port = get_arduino_port()
print("Arduino Port:", arduino_port)