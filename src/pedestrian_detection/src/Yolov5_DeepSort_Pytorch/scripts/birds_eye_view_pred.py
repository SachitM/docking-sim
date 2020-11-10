#!/usr/bin/env python3

import numpy as np

import cv2

import rospy

from sensor_msgs.msg import Image

# from autoware_msgs import DetectedObjectArray	
from visualization_msgs.msg import MarkerArray, Marker

from cam_lidar_fusion.msg import Detection, Detections, Projection, Projections

class BirdsEyeView():
    def __init__(self, dist2pix=30):

        self.dist2pix = dist2pix
        
        self.cluster_sub = rospy.Subscriber("/detection/l_shaped/objects_markers", MarkerArray, self.cluster_callback)
        self.detection_sub = rospy.Subscriber("pedestrian_detections", Detections, self.detection_callback)

        self.detections = None
        # self.projections = None
        self.lidar_range = 20
        self.im_size = self.lidar_range * dist2pix
        self.ped_color = (255, 0, 0)
        self.pred_color = (255, 255, 0)
        self.unknown_color = (0, 0, 255)
        self.car_color = (0, 255, 0)

        self.tracking_x = {}
        self.tracking_y = {}
        
        cv2.namedWindow("Bird's Eye View")

        # CHANGE
        self.extrinsics = np.array([[1.0, 0.0, 0.0, 0.0],
                                    [0.0, 1.0, 0.0, 0.338],
                                    [0.0, 0.0, 1.0, -1.0]], dtype=np.float64)

        self.intrinsics = np.array([[256.0, 0.0,    256.0],
                                    [0.0,   256.0,  256.0],
                                    [0.0,   0.0,    1.0]], dtype=np.float64)
            # using skew as (pixels/2) / tan(fov/2)



    def detection_callback(self, detections):
        # print("Detections")
        self.detections = detections.detections

    def projection_to_bev(self, projections):
        bev_image = np.ones(shape=(self.im_size, self.im_size, 3)) * 255.0
        bev_image = cv2.circle(bev_image, (self.im_size//2, self.im_size//2), 10, self.car_color, -1)

        for p in projections.projections:
            # print(p.world_point)
            coordinates = ( self.im_size//2 + int(p.world_point.x * self.dist2pix) , self.im_size//2 - int(p.world_point.y * self.dist2pix))
            # print("Coordinates", coordinates)
            if self.detections is None:
                return
            for d in self.detections:
                if d.left < p.row and (d.left + d.w) > p.row and d.top < p.col and (d.top + d.h) > p.col:
                    bev_image = cv2.circle(bev_image, coordinates, 7, self.ped_color, -1)
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    bev_image = cv2.putText(bev_image, str(d.identity), coordinates, font, 1, (0,0,0), 2, cv2.LINE_AA)

                    if str(d.identity) not in self.tracking_x:
                        self.tracking_x[str(d.identity)] = []
                        self.tracking_y[str(d.identity)] = []
                    
                    self.tracking_x[str(d.identity)].append(coordinates[0])
                    self.tracking_y[str(d.identity)].append(coordinates[1])

                    if len(self.tracking_x[str(d.identity)]) > 3:
                        pred_coords = self.predict(str(d.identity))
                        bev_image = cv2.circle(bev_image, pred_coords, 5, self.pred_color, -1)

                    break
                else:
                    bev_image = cv2.circle(bev_image, coordinates, 7, self.unknown_color, -1)
        
        cv2.imshow("Bird's Eye View", bev_image)
        cv2.waitKey(10)

    
    def predict(self, id):
        x = self.tracking_x[id]
        y = self.tracking_y[id]
        
        x, y = np.array(x)[-4:], np.array(y)[-4:]
        time = np.arange(len(x))

        print(x, y, time)

        zx = np.polyfit(time, x, 2)
        fx = np.poly1d(zx)

        zy = np.polyfit(time, y, 2)
        fy = np.poly1d(zy)

        t_new = len(x)

        

        x_pred = int(fx(t_new))
        y_pred = int(fy(t_new))

        print(x_pred, y_pred)

        return x_pred, y_pred





    def cluster_callback(self, markers):
        #process message type

        projections = Projections()

        for marker in markers.markers:
            point = marker.pose.position
            point = np.array([-point.y, point.z, point.x, 1.0]).reshape((4, 1))
            # print("Point", point)
            
            if point[0] == 0.0 or point[1] == 0.0:
                break
            # print(point)
            
            img_coords = self.intrinsics @ self.extrinsics @ point
            img_coords = img_coords // img_coords[2] + 1e-8
            # print("IMG coords", img_coords)
            proj = Projection()
            proj.row = img_coords[0]
            proj.col = img_coords[1]
            # print("Project4ed", proj.row, proj.col)
            proj.world_point.x, proj.world_point.z, proj.world_point.y = point[0], point[1], point[2]
            
            projections.projections.append(proj)

        self.projection_to_bev(projections)


if __name__ == "__main__":
    bev = BirdsEyeView()
    rospy.init_node('bev', anonymous=True)
    rospy.spin()