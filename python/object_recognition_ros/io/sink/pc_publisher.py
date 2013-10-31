"""
Module defining several outputs for the object recognition pipeline
"""

from ecto import BlackBoxCellInfo, BlackBoxForward
from object_recognition_core.boost.interface import ObjectDbParameters
from object_recognition_core.io.sink import SinkBase
from object_recognition_ros import init_ros
from object_recognition_ros.ecto_cells.io_ros import MsgAssembler, VisualizationMsgAssembler
import ecto
import ecto_ros.ecto_geometry_msgs as ecto_geometry_msgs
import ecto_ros.ecto_std_msgs as ecto_std_msgs
import ecto_ros.ecto_sensor_msgs as ecto_sensor_msgs

#PoseArrayPub = ecto_geometry_msgs.Publisher_PoseArray
#MarkerArrayPub = Publisher_MarkerArray
#StringPub = ecto_std_msgs.Publisher_String

#PointCloudPub = ecto_sensor_msgs.Publisher_PointCloud2

########################################################################################################################

class PointCloudPublisher(ecto.BlackBox, SinkBase):
    """Class publishing the different results of object recognition as ROS topics
    """
    def __init__(self, *args, **kwargs):
        init_ros()
        ecto.BlackBox.__init__(self, *args, **kwargs)
        SinkBase.__init__(self)

    @staticmethod
    def declare_cells(_p):
        return {'msg_assembler': BlackBoxCellInfo(MsgAssembler)}

    @staticmethod
    def declare_direct_params(p):
        p.declare('latched', 'Determines if the topics will be latched.', True)
        p.declare('cluster_pointcloud_topic', 'The Topic for the Cluster PointCloud', 'cluster_pointcloud')

    @staticmethod
    def declare_forwards(_p):
        #p = {'msg_assembler': [BlackBoxForward('publish_clusters')]}
        i = {'msg_assembler': 'all'}

        return ({},i,{})

    def configure(self, p, _i, _o):
        #self._recognized_object_array = Publisher_RecognizedObjectArray(topic_name=p.recognized_object_array_topic, latched=p.latched)
        self._clusters_pointcloud = ecto_sensor_msgs.Publisher_PointCloud2(topic_name=p.cluster_pointcloud_topic, latched=p.latched)

    def connections(self, p):
        # connect to a publishing cell
        connections = [ self.msg_assembler['msg'] >> self._clusters_pointcloud['input']]
        return connections
