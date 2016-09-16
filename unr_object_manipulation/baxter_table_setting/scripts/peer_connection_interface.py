#!/usr/bin/env python

import zmq
import rospy
import threading
from robotics_task_tree_eval.msg import *
import pickle

def enum(*sequential, **named):
    enums = dict(zip(sequential, range(len(sequential))), **named)
    reverse = dict((value, key) for key, value in enums.iteritems())
    enums['reverse_mapping'] = reverse
    return type('Enum', (), enums)

def SubThread(sub, cb, event):
    count = 0
    timeout = False
    while event.is_set():
        try:
            [address, msg] = sub.recv_multipart()
            cb(msg, address)
            timeout = False
            count += 1
        except zmq.ZMQError as e:
            if e.errno == zmq.ETERM:
                break
            elif e.errno == zmq.EAGAIN:
                timeout = True
            else:
                raise

class NodePeerConnectionInterface:
    def __init__(self, server_params, running_event):
        self.server_params = server_params
        self.context = zmq.Context()
        self.running_event = running_event

        self.pub = self.CreateZMQPub()
        self.ros_pubs = dict()
        self.ros_subs = dict()
        self.subs = dict()

    def InitializeSubscriber(self, name):
        # Setup peer listener
        topic = '%s%s'%(name, '_peer')
        self.ros_subs[topic] = rospy.Subscriber(topic, ControlMessage, self.ReceiveFromPeer, queue_size=5, callback_args=topic)

    def InitializePublisher(self, name):
        # setup peer publisher
        topic = '%s%s'%(name, '_peer')
        self.ros_pubs[topic] = rospy.Publisher(topic, ControlMessage, queue_size=5)

        # setup zeromq subscriber
        self.subs[topic] = self.CreateZMQSub(topic, self.SendToPeer)

    def ReceiveFromPeer(self, msg, topic):
        # Publish to server
        print 'Received from peer topic: %s'%(topic)
        self.pub.send_multipart([topic, pickle.dumps(msg)])

    def SendToPeer(self, msg, topic):
        print 'send to peer topic: %s'%(topic)
        self.ros_pubs[topic].publish(pickle.loads(msg))
        

    def CreateZMQPub(self):
        publisher = self.context.socket(zmq.PUB)
        publisher.bind("tcp://*:%s"%(self.server_params.pub_port))
        return publisher

    def CreateZMQSub(self, topic, cb):
        subscriber = self.context.socket(zmq.SUB)
        subscriber.connect("tcp://%s:%s"%(self.server_params.address, self.server_params.sub_port))
        subscriber.setsockopt(zmq.SUBSCRIBE, topic)
        subscriber.setsockopt(zmq.RCVTIMEO, 1000)

        # Create thread
        thread = threading.Thread(target=SubThread, args=[subscriber, cb, self.running_event])
        thread.start()

        return subscriber

    def AddNode(self, name):
        self.InitializeSubscriber(name)
        self.InitializePublisher(name)
    def AddOutputNode(self, name):
        self.InitializeSubscriber(name)
    def AddInputNode(self, name):
        self.InitializePublisher(name)



ROBOT = enum('PR2', 'BAXTER')
ROBOT_DICT = {'PR2' : 0, 'BAXTER' : 1}
def main():
    # Initialize Node
    rospy.init_node('interface')
    class ServerParam:
        def __init__(self):
            pass
    server = ServerParam
    server.sub_port = rospy.get_param('~sub_port', '5565')
    server.pub_port = rospy.get_param('~pub_port', '5566')
    server.address = 'localhost'
    running_event = threading.Event()
    running_event.set()
    print 'Creating Nodes'

    node_list = rospy.get_param('~NodeList', None)
    task_file = rospy.get_param('~Nodes', None)
    robot = rospy.get_param('~robot', None)
    robot = ROBOT_DICT[robot]

    interface = NodePeerConnectionInterface(server, running_event)
    for node in node_list:
        if task_file[node]['mask']['robot'] != robot:
            interface.AddOutputNode(node)
        else:
            interface.AddInputNode(node)

    print 'Spinning'
    rospy.spin()
    print 'shutting Down node'
    if rospy.is_shutdown():
        print 'Clearing event'
        running_event.clear()

if __name__ == '__main__':
    main()
