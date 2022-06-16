"""
Stand-ins for `rospy.msproxy` in ROS2.

------------------------------------------------------------------------------
This file is part of rosros - simple unified interface to ROS1 / ROS2.
Released under the BSD License.

@author      Erki Suurjaak
@created     30.05.2022
@modified    30.05.2022
------------------------------------------------------------------------------
"""
## @namespace rosros.rospify.msproxy
from .. import ros2
from .. import util


class MasterProxy:
    """Partial stand-in for `rospy.MasterProxy`"""

    def __init__(self, node):
        """
        @param   node  rclpy.Node instance
        """
        self._node = node

    def getSystemState(self):
        """
        Retrieve list representation of system state (i.e. publishers, subscribers, and services).

        @return: (1, "current system state", systemState).

           System state is in list representation::
             [publishers, subscribers, services].

           publishers is of the form::
             [ [topic1, [topic1Publisher1...topic1PublisherN]] ... ]

           subscribers is of the form::
             [ [topic1, [topic1Subscriber1...topic1SubscriberN]] ... ]

           services is of the form::
             [ [service1, []] ... ]
        """
        pubs, subs, srvs = [], [], []
        for topic, _ in self._node.get_topic_names_and_types():
            nodes = []
            for info in self._node.get_publishers_info_by_topic(topic):
                nodes.append(util.namejoin(info.node_namespace, info.node_name))
            pubs.append([topic, sorted(nodes)])
            nodes = []
            for info in self._node.get_subscriptions_info_by_topic(topic):
                nodes.append(util.namejoin(info.node_namespace, info.node_name))
            subs.append([topic, sorted(nodes)])

        srvnodes = {}  # {service name: [node, ]}
        for name, ns in self._node.get_node_names_and_namespaces():
            fullname = util.namejoin(ns, name)
            for srv, _ in self._node.get_service_names_and_types_by_node(name, ns):
                srvnodes.setdefault(srv, []).append(fullname)
        srvs = [[srv, sorted(nodes)] for srv, nodes in sorted(srvnodes.items())]

        return 1, "current system state", [sorted(pubs), sorted(subs), srvs]

    def getPublishedTopics(self, subgraph=""):
        """
        Get list of topics that can be subscribed to.

        This does not return topics that have no publishers.
        See `getSystemState()` to get more comprehensive list.

        @param   subgraph  return only topics under subgraph, e.g. "/foo" will match
                           "/foo/bar" but not "/foobar". "" returns all topics.
        @return:           (1, "current topics", [[topicName, typeName], ])
        """
        pairs = set()  # {(topic, typename)}
        if subgraph: subgraph = subgraph.rstrip("/") + "/"
        for topic, _ in self._node.get_topic_names_and_types():
            if subgraph and not topic.startswith(subgraph): continue  # for topic

            for info in self._node.get_publishers_info_by_topic(topic):
                pairs.add((topic, ros2.canonical(info.topic_type)))
        return 1, "current topics", [list(x) for x in sorted(pairs)]

    def getTopicTypes(self):
        """
        Returns a list of topic names and their types.

        @return   (1, "current system state", [[topicName, topocType], ])
        """
        result = []
        for topic, typenames in sorted(self._node.get_topic_names_and_types()):
            for typename in typenames:
                result.append([topic, ros2.canonical(typename)])
        return 1, "current system state", result


__all__ = [
    "MasterProxy"
]
