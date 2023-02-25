from rplidar import RPLidar
import time

lidar = RPLidar('/dev/ttyUSB0')


while True:
    lidar.clean_input()
    time.sleep(0.2)
    for i, scan in enumerate(lidar.iter_measures()):
        print('%d: Got %d measurments' % (i, len(scan)))
        if i > 10:
            break

lidar.stop()
lidar.stop_motor()
lidar.disconnect()