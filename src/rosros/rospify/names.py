"""
Stand-ins for `rospy.names` in ROS2.

------------------------------------------------------------------------------
This file is part of rosros - simple unified interface to ROS1 / ROS2.
Released under the BSD License.

@author      Erki Suurjaak
@created     30.05.2022
@modified    30.05.2022
------------------------------------------------------------------------------
"""
## @namespace rosros.rospify.names
from .. import ros2


def get_name():
    """Returns fully resolved name of local node."""
    return ros2.get_node_name() if ros2.NODE else "/unnamed"


## Backwards compatibility
get_caller_id = get_name


def get_namespace():
    """Returns namespace of local node."""
    return ros2.NODE.get_namespace() if ros2.NODE else "/"


def remap_name(name, caller_id=None, resolved=True):
    """
    Returns a remapped topic/service name.

    @param   name       name to remap
    @param   caller_id  namespace to resolve a relative or private name under,
                        by default current node namespace
    @param   resolved   ignored (does nothing in ROS1 also)
    @return             remapped name resolved to namespace, or original if not mapped
    """
    return ros2.remap_name(name, caller_id)


def resolve_name(name, caller_id=None):
    """
    Resolve a ROS name to its global, canonical form.

    Private ~names are resolved relative to the node name.

    @param   name       name to resolve.
    @param   caller_id  node name to resolve relative to.
                        To resolve to local namespace, omit this parameter (or use None)
    @return             Resolved name. If name is empty/None, resolve_name
                        returns parent namespace.

    """
    return ros2.resolve_name(name, caller_id)


__all__ = [
    "get_caller_id", "get_name", "get_namespace", "remap_name", "resolve_name"
]
