#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from twisted.web import xmlrpc, server
from twisted.internet import reactor, endpoints

"""
This is our Publisher, everything said will be put into Pepper command.
"""
pub = rospy.Publisher('pepper_command', String, queue_size=10)


"""
This class is our simple server, with the IP adress of this machine
and the port, we can send messages to our ROSCore
"""
class RPCServer(xmlrpc.XMLRPC):

    """
    Pepper can say anything with this command. This can be used for praticaly anthing.
    Of course, one would need to parse the input to make any sense of it.
    """
    def xmlrpc_echo(self, x):
        print("Got a message: " + x)
        rospy.loginfo("Got message through pepper_communication/RPC_Server: "  + x)
        pub.publish(x)
        return x

    """
    This method is supposed to tell the PR2 to open or close its grip. True being open and false to close.
    """
    def xmlrpc_openGrip(self, bool):
        if bool == 1:
            rospy.loginfo("Got command to open gripper from outsiderpccomm: " + bool)
            print("Open")
            return
        else:
            rospy.loginfo("Got command to close gripper from outsiderpccomm: " + bool)
            print("Close")

    """
    This method is activated if someone wants cake and tells pepper that. This will just write a string into a topic.
    To communicate using more words use the ECHO command. This allows you to write anything into the topic.
    """
    def xmlrpc_getCake(self):
        print("Got a I want cake!")
        rospy.loginfo("Got I want cake! through pepper_communication/RPC_Server.")
        pub.publish("Someone wants cake!")





"""
We do need to start everything ROS-related first of course
"""
def rosstuff():
    print ("Initializing RPC_Server for connection with Pepper. Don't forget to note the IP address on this machine.")
    r = RPCServer()
    endpoint = endpoints.TCP4ServerEndpoint(reactor, 7080)
    endpoint.listen(server.Site(r))
    rospy.init_node('rpc_server', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    print("Ready to use. You just need the IP address from this machine and the xmlrpc package to use this server :)")
    rospy.loginfo("The pepper_communication/rpc_server node is ready to use.")
    reactor.run()
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    try:
        rosstuff()
    except rospy.ROSInterruptException:
        pass
