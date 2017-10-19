#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml

def _perform_voxel_grid_downsampling(cloud,leaf=0.01):
    vox = cloud.make_voxel_grid_filter()
    vox.set_leaf_size(leaf, leaf, leaf)
    return vox.filter()

def _perform_statistical_outlier_sampling(cloud, mean_k=10, thresh_scale_factor=0.1):
    outlier_filter = cloud.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(mean_k)
    # outlier = mean distance > (mean distance + thresh_scale_factor * std_dev)
    outlier_filter.set_std_dev_mul_thresh(thresh_scale_factor)
    return outlier_filter.filter()

def _perform_pass_through_filter(cloud, axis='z', v_min=0.76, v_max=1.1):
    passthrough = cloud.make_passthrough_filter()
    passthrough.set_filter_field_name(axis)
    passthrough.set_filter_limits(v_min, v_max)

    # Finally use the filter function to obtain the resultant point cloud.
    return passthrough.filter()

def _perform_ransac_plane_fitter(cloud, max_dist=0.01):
    # Create the segmentation object
    seg = cloud.make_segmenter()

    # Set the model you wish to fit
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

    seg.set_distance_threshold(max_dist)

    # Call the segment function to obtain set of inlier indices and model coefficients
    inliers, coefficients = seg.segment()
    extracted_inliers = cloud.extract(inliers, negative=False)
    extracted_outliers = cloud.extract(inliers, negative=True)
    return extracted_inliers, extracted_outliers

def _perform_euclidean_clustering(cloud):
    white_cloud = XYZRGB_to_XYZ(cloud)
    tree = white_cloud.make_kdtree()

    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold
    # as well as minimum and maximum cluster size (in points)
    # NOTE: These are poor choices of clustering parameters
    # Your task is to experiment and find values that work for segmenting objects.
    ec.set_ClusterTolerance(0.05)
    ec.set_MinClusterSize(10)
    ec.set_MaxClusterSize(10000)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()

    #Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                            white_cloud[indice][1],
                                            white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    #Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)
    return cluster_cloud, cluster_indices

def _extract_features(cloud):
    chists = compute_color_histograms(cloud, using_hsv=True)
    normals = get_normals(cloud)
    nhists = compute_normal_histograms(normals)
    features = np.concatenate((chists, nhists))
    return features

# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    # TODO: Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)

    # TODO: Statistical Outlier Filtering
    cloud = _perform_statistical_outlier_sampling(cloud)
    # TODO: Voxel Grid Downsampling
    cloud = _perform_voxel_grid_downsampling(cloud, leaf=0.005)

    # TODO: PassThrough Filter
    cloud = _perform_pass_through_filter(cloud, v_min=0.605, v_max=1.0)
    cloud = _perform_pass_through_filter(cloud, axis='y', v_min=-0.5, v_max=0.5)

    # TODO: RANSAC Plane Segmentation
    # TODO: Extract inliers and outliers
    cloud_table, cloud_objects = _perform_ransac_plane_fitter(cloud)

    # TODO: Euclidean Clustering
    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    cloud_clusters, cluster_indices = _perform_euclidean_clustering(cloud_objects)

    # TODO: Convert PCL data to ROS messages
    # TODO: Publish ROS messages
    pcl_objects_pub.publish(pcl_to_ros(cloud_objects))
    pcl_table_pub.publish(pcl_to_ros(cloud_table))
    pcl_clusters_pub.publish(pcl_to_ros(cloud_clusters))

# Exercise-3 TODOs:

    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects = []
    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster
        pcl_cluster = cloud_objects.extract(pts_list)
        white_cloud = XYZRGB_to_XYZ(cloud)
        # Compute the associated feature vector
        # TODO: convert the cluster from pcl to ROS using helper function
        ros_cluster = pcl_to_ros(pcl_cluster)

        feature = _extract_features(ros_cluster)

        # Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)
        # Publish a label into RViz
        points_arr = list(pcl_cluster)
        label_pos = np.mean(points_arr, axis=0)[:3]
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    # Publish the list of detected objects
    detected_objects_pub.publish(detected_objects)

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException as e:
        print(e)
        pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # TODO: Initialize variables
    TEST_SCENE_NUM = Int32()
    OBJECT_NAME = String()
    WHICH_ARM  = String()
    PICK_POSE = Pose()
    PLACE_POSE = Pose()


    # TODO: Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    dropbox_param = rospy.get_param('/dropbox')
    test_scene_num = rospy.get_param('/test_scene_num')

    TEST_SCENE_NUM.data = test_scene_num

    # TODO: Parse parameters into individual variables
    labels = []
    centroids = [] # to be list of tuples (x, y, z)
    for object in object_list:
        labels.append(object.label)
        points_arr = ros_to_pcl(object.cloud).to_array()
        centroids.append(np.mean(points_arr, axis=0)[:3])

    dropbox_positions = {x['group']: x['position'] for x in dropbox_param}

    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # TODO: Loop through the pick list
    dict_list = []
    for i in range(len(object_list_param)):
        object_name = object_list_param[i]['name']
        object_group = object_list_param[i]['group']

        OBJECT_NAME.data = object_name

        # TODO: Get the PointCloud for a given object and obtain it's centroid
        centroid = centroids[labels.index(object_name)]
        PICK_POSE.position.x = np.asscalar(centroid[0])
        PICK_POSE.position.y = np.asscalar(centroid[1])
        PICK_POSE.position.z = np.asscalar(centroid[2])


        # TODO: Create 'place_pose' for the object
        PLACE_POSE.position.x = dropbox_positions[object_group][0]
        PLACE_POSE.position.y = dropbox_positions[object_group][1]
        PLACE_POSE.position.z = dropbox_positions[object_group][2]

        # TODO: Assign the arm to be used for pick_place
        WHICH_ARM.data = 'right' if object_group == 'green' else 'left'

        # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
        obj_yaml = make_yaml_dict(
                TEST_SCENE_NUM,
                WHICH_ARM,
                OBJECT_NAME,
                PICK_POSE,
                PLACE_POSE)
        dict_list.append(obj_yaml)

        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')

        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # TODO: Insert your message variables to be sent as a service request
            resp = pick_place_routine(
                TEST_SCENE_NUM,
                OBJECT_NAME,
                WHICH_ARM,
                PICK_POSE,
                PLACE_POSE)

            print ("Response: ",resp.success)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    # TODO: Output your request parameters into output yaml
    print('now trying to save file')
    send_to_yaml('output_{}.yaml'.format(test_scene_num), dict_list)
    # exit(0)


if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('pick_placing', anonymous=True)
    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)
    # TODO: Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_clusters_pub = rospy.Publisher("/pcl_clusters", PointCloud2, queue_size=1)
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)
    # TODO: Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
