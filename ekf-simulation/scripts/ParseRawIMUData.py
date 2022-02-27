import re


def SecSubsecToDouble(seconds, subseconds):
    return seconds + subseconds * (2 ** -32)


def parseLine(line):
    tokens = re.split("\s+", line)
    seconds = float(tokens[1])
    subseconds = float(tokens[2])
    gyroscope = [float(gyro_val) for gyro_val in tokens[3:6]]
    acceleromter = [float(accel_val) for accel_val in tokens[6:9]]
    inclinomter = [float(inclin_val) for inclin_val in tokens[9:12]]
    return SecSubsecToDouble(seconds, subseconds), gyroscope, acceleromter, inclinomter


def parseIMUFile(filePath):
    option = input(
        "Simulate IMU sending (only enter \"1\" \"2\" or \"3\"):\n"
        "1. Only unique messages with increasing timestamps\n"
        "2. Also send duplicate messages\n"
        "3. Also send messages with non-montonic timestamps\n")

    time = []
    gyro = []
    accel = []
    inclin = []

    with open(filePath, 'r') as file:
        lines = file.readlines()
        for line in lines:
            if "[First]" in line or "[Good]" in line:
                if option in ["1", "2", "3"]:
                    timestamp, gyro_val, accel_val, inclin_val = parseLine(line)
                    time.append(timestamp)
                    gyro.append(gyro_val)
                    accel.append(accel_val)
                    inclin.append(inclin_val)
            elif "BIG dt" in line:
                if option in ["3"]:
                    timestamp, gyro_val, accel_val, inclin_val = parseLine(line)
                    time.append(timestamp)
                    gyro.append(gyro_val)
                    accel.append(accel_val)
                    inclin.append(inclin_val)
            else:
                print(f"Line not parseable: \"{line}\"")

    return time, gyro, accel, inclin
