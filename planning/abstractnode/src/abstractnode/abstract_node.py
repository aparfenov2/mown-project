import rospy


class AbstractNode(object):
    def __init__(self, node_name, rate):
        rospy.init_node(node_name)
        self.initialization()
        self._sleep = rospy.Rate(rate)

    def initialization(self):
        pass

    def run(self):
        while not rospy.is_shutdown():
            try:
                self.work()
                self._sleep.sleep()
            except Exception as e:
                pass

    def work(self):
        pass