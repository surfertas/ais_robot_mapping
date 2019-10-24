#!/usr/bin/python3

from tools.dataobjects import Sensor, Odometry, Data


def read_data(filename: str) -> Data:
    """ Reads the odometry and sensor reading from a file. """
    data = Data()
    first = data.first
    sensor = []

    with open(filename, 'r') as f:
        for line in f:
            fields = line.split(' ')
            data_type = fields[0]

            if data_type == 'ODOMETRY':
                if not first:
                    data.timestep.append((odom, sensor))
                    sensor = []

                first = False
                odom = Odometry(*[float(field) for field in fields[1:]])
            if data_type == 'SENSOR':
                sensor.append(Sensor(int(fields[1]), float(fields[2]), float(fields[3])))

    return data
