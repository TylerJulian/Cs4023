#!/usr/bin/env python
import rospy

from project2_dispatcher import Dispatcher


def main():

    rospy.init_node('main_node')
    dispatcher = Dispatcher()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
