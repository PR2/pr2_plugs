#!/usr/bin/env python
# this script reads the parameters vision_plug_detection/plug_position_x and vision_plug_detection/plug_position_y
# and adds a calibration offset to these parameters

import roslib; roslib.load_manifest('pr2_plugs_actions')
import sys
import os,stat
import yaml
def save_config(filename, config):
    file = open(filename, "w")
    for k,v in config.items():
        file.write(" " + k + ": " + str(v) + "\r\n")
    file.close()
    os.chmod(filename,stat.S_IRUSR | stat.S_IXUSR | stat.S_IWUSR | stat.S_IROTH | stat.S_IXOTH | stat.S_IWOTH)

def main():
    param_ns = None
    param_x = 'plug_position_x'
    param_z = 'plug_position_z'

    if len(sys.argv) != 2:
        print 'Usage: ./load_plugin_calibration.py file.yaml'
        return 1

    # read yaml file
    config = None
    try:
        config = yaml.load(open(sys.argv[1]))
    except:
        print 'Could not load yaml file %s'%sys.argv[1]
        return 1


    # runnin on HW
    if os.getenv('ROBOT') == 'pr2':
        try:
            offset = yaml.load(open('/etc/ros/plugs/calibration.yaml'))
            config[param_x] = config[param_x] + offset[param_x]
            config[param_z] = config[param_z] + offset[param_z]
            save_config("/tmp/plugs_config.yaml", config)
            print 'loaded plug calibraiton offset parameters from /etc/ros/plugs/calibration.yaml'
        except Exception,ex:
            print 'Cannot load yaml /etc/ros/plugs/calibration.yaml'
            raise ex
            return 1

    # running in SIM
    elif os.getenv('ROBOT')== 'sim':
        print "Using sim config"
        save_config("/tmp/plugs_config.yaml", config)

    # robot env variable not set
    else:
        print 'no ROBOT env set, cannot load plug calibration offset'
        return 1
    
    
    return 0


if __name__ == '__main__':
    sys.exit(main())

