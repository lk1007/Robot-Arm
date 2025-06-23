#global packages
import numpy as np
from scipy.stats import trim_mean
import cv2
import sys
import warnings
import torch
from typing import Tuple
#libfreenect
from pylibfreenect2 import Freenect2, SyncMultiFrameListener
from pylibfreenect2 import FrameType, Registration, Frame
from pylibfreenect2 import createConsoleLogger, setGlobalLogger
from pylibfreenect2 import LoggerLevel
#ros2
import rclpy
from rclpy.node import Node
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import Image, CameraInfo, PointCloud2

from cv_bridge import CvBridge
from std_msgs.msg import Header

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

from vision_msgs.msg import Detection3D, ObjectHypothesisWithPose
from geometry_msgs.msg import PoseWithCovariance, Pose
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray

class KinectNode(Node):
    COLOR_WIDTH, COLOR_HEIGHT = (1920,1080)
    DEPTH_WIDTH, DEPTH_HEIGHT = (512,424)
    SCALE_X = DEPTH_WIDTH/COLOR_WIDTH
    SCALE_Y = DEPTH_HEIGHT/COLOR_HEIGHT

    color_with_detections = np.empty(0)
    registered_with_detections = np.empty(0)
    bigdepth = np.zeros([COLOR_WIDTH,COLOR_HEIGHT])
    bigdepth_with_detections = np.empty(0)
    undistortedFrame = Frame(DEPTH_WIDTH,DEPTH_HEIGHT, 4)
    registeredFrame = Frame(DEPTH_WIDTH, DEPTH_HEIGHT, 4)
    bigdepthFrame = Frame(COLOR_WIDTH,(COLOR_HEIGHT+2),4)
    color = np.zeros([])

    def __init__(self):
        super().__init__('kinect_node')
        #disable warnings
        warnings.simplefilter("ignore", FutureWarning)
        #parameters
        self.declare_parameter('publish_pointcloud', False)
        self.declare_parameter('publish_detection', False)
        #Create pipeline
        try:
            from pylibfreenect2 import OpenGLPacketPipeline
            pipeline = OpenGLPacketPipeline()
        except:
            try:
                from pylibfreenect2 import OpenCLPacketPipeline
                pipeline = OpenCLPacketPipeline()
            except:
                from pylibfreenect2 import CpuPacketPipeline
                pipeline = CpuPacketPipeline()

        if(self.get_parameter('publish_detection').get_parameter_value().bool_value):
            #load object detection model
            self.model : torch.nn.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
        #Setup publishers
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', 10)
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/pointcloudkinect', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, 'camera/camera_info', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        self.detection_pub = self.create_publisher(Detection3D, '/detections_3d', 10)


        self.bridge = CvBridge()

        # Create and set logger
        self.logger = createConsoleLogger(LoggerLevel.Debug)
        setGlobalLogger(self.logger)

        # setup kinect device
        self.fn = Freenect2()
        num_devices = self.fn.enumerateDevices()
        if num_devices == 0:
            print("No device connected!")
            sys.exit(1)

        self.serial = self.fn.getDeviceSerialNumber(0)
        self.device = self.fn.openDevice(self.serial, pipeline=pipeline)

        #setup frame listener
        self.listener = SyncMultiFrameListener(
            FrameType.Color | FrameType.Ir | FrameType.Depth)
        self.device.setColorFrameListener(self.listener)
        self.device.setIrAndDepthFrameListener(self.listener)

        #start kinect
        self.device.start()

        self.registration = Registration(self.device.getIrCameraParams(),
                                    self.device.getColorCameraParams())


        self.timer = self.create_timer(0.1, self.loop_func)
    def getPointXYZ(self, y, x):
        x = int(x)
        y = int(y)
        intrinsics = self.device.getColorCameraParams()
        # Check bounds
        #if not (0 <= x < self.bigdepth.shape[1] and 0 <= y < self.bigdepth.shape[0]):
        #    print("AHHH")
        #    return np.nan,np.nan,np.nan  # Invalid depth

        z_mm = self.bigdepth[y, x]
        if z_mm == np.nan:
            return np.nan,np.nan,np.nan  # Invalid depth

        z = z_mm / 1000.0  # convert to meters

        fx, fy, cx, cy = intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy
        x3d = (x - cx) * z / fx
        y3d = (y - cy) * z / fy

        return (x3d, y3d, z)
    def get_frame_blocking(self):
        if(not self.listener.hasNewFrame()):
            return None
        frames = self.listener.waitForNewFrame()
        pre_color = frames["color"]
        self.color = pre_color.asarray()
        self.color = cv2.cvtColor(self.color, cv2.COLOR_BGRA2BGR)
        self.ir = frames["ir"]
        self.ir = self.ir.asarray() / 65535.
        pre_depth = frames["depth"]
        self.depth = pre_depth.asarray()
        self.registration.apply(pre_color, pre_depth, self.undistortedFrame, self.registeredFrame, enable_filter=True, bigdepth=self.bigdepthFrame, color_depth_map=None)
        self.bigdepth = (self.bigdepthFrame.asarray(np.float32)[:1080, :]).copy()
        #self.registered = self.registeredFrame.asarray(np.uint8)
        self.listener.release(frames)
        return True
    def publish_object_detection(self):
        def detection_msg_to_marker_arr(detection_msg):
            def detection_to_marker(detectionRosMsg: Detection3D, idx: int) -> Marker:
                marker = Marker()
                marker.header = detectionRosMsg.header
                marker.ns = "detections"
                marker.id = idx
                marker.type = Marker.CUBE
                marker.action = Marker.ADD

                # Use the bbox center and size from Detection3D
                marker.pose = detectionRosMsg.bbox.center
                marker.scale.x = detectionRosMsg.bbox.size.x
                marker.scale.y = detectionRosMsg.bbox.size.y
                marker.scale.z = detectionRosMsg.bbox.size.z

                # Color: e.g., green with some alpha
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 0.5

                marker.lifetime.sec = 0  # 0 = forever
                return marker
            marker_array = MarkerArray()
            marker_array.markers.append(detection_to_marker(detection_msg,0))
            return marker_array
        def to_ros_topic(xd,yd,wd,hd,confidence):
            def find_avg_depth(center_x,center_y,width,height):
                test_x = np.linspace(int(center_x-width/2),int(center_x+width/2), 10)
                test_y = np.linspace(int(center_y-height/2),int(center_y+height/2), 10)
                depth = np.zeros([10,10])
                for i,x in enumerate(test_x):
                    for j,y in enumerate(test_y):
                        _,_,depth[i,j] = self.getPointXYZ( y, x)
                x,y,_ = self.getPointXYZ(center_y,center_x)
                return x,y,trim_mean(depth[~np.isnan(depth)].flatten(), proportiontocut=.1)
            if(xd == 0 or yd == 0):
                return None
            detection = Detection3D()
            detection.header.stamp = self.timestamp
            detection.header.frame_id = 'kinect'  # or camera frame, whichever your detections are in

            # Hypothesis: class and confidence
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = 'cup'  # detected object class label
            hypothesis.hypothesis.score = confidence  # confidence
            detection.results.append(hypothesis)

            # Pose of detected object in 3D (fill with your detection center point)
            pose = PoseWithCovariance()

            #x,y,z = self.registration.getPointXYZ(self.undistorted,pre_y,pre_x)
            xp,yp,zp = np.nan,np.nan,np.nan
            xp,yp,zp = find_avg_depth(xd,yd,wd,hd)
            wp,hp,_ = self.getPointXYZ(yd+int(hd/2),xd+int(wd/2))
            print(xp,yp,zp, wp, hp)
            wp = abs(wp-xp)*2
            hp = abs(hp-yp)*2
            if(xp == np.nan or yp == np.nan or zp == np.nan):
                return None
            pose.pose.position.x = float(xp)
            pose.pose.position.y = float(yp)
            pose.pose.position.z = float(zp)

            # Optional: pose orientation and covariance (zero or identity)
            pose.pose.orientation.w = 1.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0

            detection.bbox.center = pose.pose  # center pose of detection bounding box

            # Fill size of bounding box (in meters)
            #detection.bbox.size.x = float(detections.width)
            #detection.bbox.size.y = float(detections.height)
            detection.bbox.size.x = float(.1)
            detection.bbox.size.y = float(.1)
            detection.bbox.size.z = float(.1)
            return detection, xp,yp,zp
        def add_boxes(image, x,y,w,h, label = None):
            x,y,w,h = int(x),int(y),int(w),int(h)
            img = image.copy()
            cv2.rectangle(img=img, pt1=(int(x-w/2.0), int(y-h/2.0)), pt2=(int(x+w/2.0), int(y+h/2.0)), color=(0, 255, 0), thickness=2)
            if(label):
                cv2.putText(img, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 255, 0), 2, cv2.LINE_AA)
            return img
        def do_detection(image):
            results = self.model(image)
            #convert to center_x, center_y
            detection_results = results.pandas().xywh[0]
            detection_results = detection_results[detection_results.name == "cup"]
            if(detection_results.empty):
                return detection_results
            detection_results = detection_results.loc[detection_results.confidence.idxmax()]
            return detection_results
        detection = do_detection(self.color)
        if(detection.empty):
            return
        xc,yc,wc,hc = detection.xcenter,detection.ycenter,detection.width,detection.height
        self.color_with_detections = add_boxes(self.color, xc,yc,wc,hc)
        self.bigdepth_with_detections = add_boxes(self.bigdepth, xc,yc,wc,hc)
        thing = to_ros_topic(xc,yc,wc,hc,detection.confidence)
        if(not thing):
            return
        detection_msg,xp,yp,zp = thing
        self.detection_pub.publish(detection_msg)
        self.marker_pub.publish(detection_msg_to_marker_arr(detection_msg))

    def publish_frames(self):
        image = self.color_with_detections
        msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        msg.header.stamp = self.timestamp
        msg.header.frame_id = 'kinect'

        self.camera_info =  CameraInfo()
        self.camera_info.width = image.shape[1]
        self.camera_info.height = image.shape[0]
        params = self.device.getColorCameraParams()
        self.camera_info.k = [params.fx, 0.0, params.cx,
             0.0, params.fy, params.cy,
             0.0, 0.0, 1.0]
        self.camera_info.r = [1.0, 0.0, 0.0,
             0.0, 1.0, 0.0,
             0.0, 0.0, 1.0]
        self.camera_info.p = [params.fx, 0.0, params.cx, 0.0,
                0.0, params.fy, params.cy, 0.0,
                0.0, 0.0, 1.0, 0.0]
        self.camera_info.header = msg.header

        self.image_pub.publish(msg)
        self.camera_info_pub.publish(self.camera_info)

    def publish_pointcloud(self):
        header = Header()
        header.stamp = self.timestamp
        header.frame_id = 'kinect'
        out = np.zeros((self.depth.shape[0]*self.depth.shape[1], 3)) #shape = (217088, 3)
        for row in range(self.depth.shape[0]):
            for col in range(self.depth.shape[1]):
                world = self.registration.getPointXYZ(self.undistortedFrame, row, col) #convert depth pixel to real-world coordinate
                if(world != np.nan,np.nan,np.nan):
                    out[row*self.depth.shape[1] + col] = world
        msg = pc2.create_cloud_xyz32(header, out)
        self.pointcloud_pub.publish(msg)

    def display_frames(self):
        # NOTE for visualization:
        # cv2.imshow without OpenGL backend seems to be quite slow to draw all
        # things below. Try commenting out some imshow if you don't have a fast
        # visualization backend.
        #self.undistorted_img = cv2.normalize(self.undistorted.asarray(np.float32), None, 0, 255, cv2.NORM_MINMAX)
        #self.undistorted_img = self.undistorted_img.astype(np.uint8)

        cv2.imshow("bigdepth", self.bigdepth)
        #registered_bgr = cv2.cvtColor(self.registered, cv2.COLOR_BGRA2BGR)
        #if(self.registered_with_detections.size > 0):
        #    cv2.imshow("registered", self.registered_with_detections)
        #cv2.imshow("color", self.color)

        key = cv2.waitKey(delay=2)
        if key == ord('q'):
            return False
        return True

    def loop_func(self):
        if (not self.get_frame_blocking()):
            return
        self.timestamp = self.get_clock().now().to_msg()
        if(self.get_parameter('publish_pointcloud').get_parameter_value().bool_value):
            self.publish_pointcloud()
        if(self.get_parameter('publish_detection').get_parameter_value().bool_value):
            self.publish_object_detection()
        #self.display_frames()
        self.publish_frames()

    def cleanup(self):
        if self.timer is not None:
            self.timer.cancel()
        self.device.stop()
        self.device.close()


def main(args = None):
    rclpy.init(args=args)
    node = KinectNode()

    try:
        while rclpy.ok():
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        print("Shutting down gracefully...")
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
if __name__ == "__main__":
    main()