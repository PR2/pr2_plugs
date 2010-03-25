#!/usr/bin/env python
# this script reads the parameters vision_plug_detection/plug_position_x and vision_plug_detection/plug_position_y
# and adds a calibration offset to these parameters

import roslib; roslib.load_manifest('pr2_plugs_actions')
import sys
import os
import rospy
import yaml

def main():
    rospy.init_node('plugin_calibration_loader')
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
        for ns in config:
            param_ns = ns
        config = config[param_ns]
    except:
        print 'Could not load yaml file %s'%sys.argv[1]
        return 1


    # runnin on HW
    if os.getenv('ROBOT') == 'pr2':
        try:
            offset = yaml.load(open('/etc/ros/plugs/calibration.yaml'))
            config[param_x] = config[param_x] + offset[param_x]
            config[param_z] = config[param_z] + offset[param_z]
            rospy.set_param(param_ns, config)
            print 'loaded plug calibraiton offset parameters from /etc/ros/plugs/calibration.yaml'
        except:
            print 'cannot load yaml /etc/ros/plugs/calibration.yaml'
            return 1

    # running in SIM
    elif os.getenv('ROBOT')== 'sim':
        print 'set plug calibration offset to 0'

    # robot env variable not set
    else:
        print 'no ROBOT env set, cannot load plug calibration offset'
        return 1

    return 0


if __name__ == '__main__':
    sys.exit(main())

