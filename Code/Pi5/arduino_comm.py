import serial

ARDUINO_PORT = '/dev/ttyUSB0'
ARDUINO_BAUD = 115200

arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=1)


def transmetteur_arduino(param1="", param2="", param3=""):
    command = f"[{param1}],[{param2}],[{param3}]\n"
    arduino.write(command.encode())
    return command


_last_sent = {"steering": None, "rpm": None}


def envoyer_si_nouveau(steering=None, rpm=None):
    """
    Only sends a command to the Arduino if steering or rpm actually
    changed since the last call — avoids spamming identical commands
    every frame when nothing new needs to happen.
    """
    if steering == _last_sent["steering"] and rpm == _last_sent["rpm"]:
        return None

    _last_sent["steering"] = steering
    _last_sent["rpm"] = rpm

    steering_str = "" if steering is None else steering
    rpm_str = "" if rpm is None else rpm

    return transmetteur_arduino(param1=rpm_str, param2=steering_str)
