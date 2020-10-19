#!/usr/bin/env python


import rospy

from project2_dispatcher import Dispatcher


def main():
    # initiate main node
    rospy.init_node('main_node', anonymous=False, log_level=rospy.WARN)

    # initiate dispatcher
    dispatcher = Dispatcher()
    # register the dispatcher kill to fired when exiting
    rospy.on_shutdown(lambda: dispatcher.kill())
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
