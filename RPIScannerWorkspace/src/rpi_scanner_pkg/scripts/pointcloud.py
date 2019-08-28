from math import sin, cos, floor, radians,ceil, pi
import point_cloud2 as pc2
from std_msgs.msg import Header
import rospy
class PointCloud:
    def __init__(self, frame, rawhfov, desiredhfov, vfov):
        self.frame = frame
        self.desiredhfov = float(desiredhfov)
        self.vfov = float(vfov)
        self.rawhfov = rawhfov
        self.__parseFrame()

    def __parseFrame(self):
        print("pointcloud: starting the frame parsing")
        pointArray = []
        for (y, fullRow) in enumerate(self.frame):
            # trim hfov
            pixelWidth = len(fullRow)
            ratio = self.desiredhfov / self.rawhfov
            newPixelWidth = ratio * pixelWidth
            pixelsToRemove = pixelWidth - newPixelWidth

            row = fullRow[int(ceil(pixelsToRemove/2)) : pixelWidth-int(ceil(pixelsToRemove/2)) ]
            for x, depth in enumerate(row):
                if(depth == 0):
                    continue
                thetax = radians((self.rawhfov / float(len(fullRow)))
                                    * (pixelsToRemove+x)) - self.rawhfov/2
                thetay = radians(
                    (self.vfov / float(len(self.frame))) * y) - self.vfov/2
                vx = float(sin(thetax) * depth)
                vy = float(sin(thetay) * depth)
                vz = abs(float(cos(thetay) * depth))
                vector3 = [vx, vy, vz]
                pointArray.append(vector3)
        self.points = pointArray
        print("pointcloud: ended the frame parsing")

    def __parseFrameSpherical(self):
        """
        uses a different set of math to transform depth value to point
        NOTE: gives odd results. it almost works great, but the frame is twisted..
        """
        print("pointcloud: starting the frame parsing")
        pointArray = []
        for (y, row) in enumerate(self.frame):
            #trim hfov
            pixelWidth = len(row)
            deltaAngle = self.rawhfov - self.desiredhfov
            pixelsToRemove = pixelWidth / deltaAngle 
            row = row[int(ceil(pixelsToRemove/2)) : pixelWidth-int(ceil(pixelsToRemove/2)) ]
            
            for x, depth in enumerate(row):
                if(depth == 0):
                    continue
                width = float(len(row))
                height = float(len(self.frame))
                thetax = (pi/2) - radians((self.rawhfov / width)  * x)
                thetay = (pi/2) - radians((self.vfov / height) * y)
                vx = float(cos(thetax) * sin(thetay)  * depth)
                vy = float(sin(thetax) * sin(thetay) * depth)
                vz = float(cos(thetay) * depth)
                vector3 = [vx, vy, vz]
                pointArray.append(vector3)
        self.points = pointArray
        print("pointcloud: ended the frame parsing")

    def toPCD(self):
        outputList = []
        pointCount = len(self.points)
        outputList.append('# .PCD v.7 - Point Cloud Data file format')
        outputList.append('VERSION .7')
        outputList.append('FIELDS x y z')
        outputList.append('SIZE 4 4 4')
        outputList.append('TYPE F F F')
        outputList.append('WIDTH {}'.format(pointCount))
        outputList.append("HEIGHT 1")
        outputList.append('VIEWPOINT 0 0 0 1 0 0 0')
        outputList.append('POINTS {}'.format(pointCount))
        outputList.append('DATA ascii')
        for p in self.points:
            outputList.append(' '.join(map(str, p)))
        return '\n'.join(outputList)
    
    def toPointCloud2(self):
        data = Header()
        data.stamp = rospy.Time.now()
        data.frame_id = "lidar_link"
        return pc2.create_cloud_xyz32(data,self.points)