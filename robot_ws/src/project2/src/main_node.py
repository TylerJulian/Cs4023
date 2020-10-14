#!/usr/bin/env python
import rospy

from project2_dispatcher import Dispatcher


def main():
    # initiate main node
    rospy.init_node('main_node')

    # initiate dispatcher
    dispatcher = Dispatcher()
    while not rospy.is_shutdown():
        pass
    dispatcher.kill()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
