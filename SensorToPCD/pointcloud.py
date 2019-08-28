from typedefinitions import Frame, PointArray
from math import sin, cos, floor, radians


class PointCloud:
    def __init__(self, frame: Frame, hfov: int, vfov: int):
        self.frame: Frame = frame
        self.hfov = float(hfov)
        self.vfov = float(vfov)
        self.__parseFrame()

    def __parseFrame(self) -> PointArray:
        pointArray: PointArray = []
        for (y, row) in enumerate(self.frame):
            for x, depth in enumerate(row):
                if(depth == 0):
                    continue
                thetax = radians((self.hfov / float(len(row)))
                                 * x) - self.hfov/2
                thetay = radians(
                    (self.vfov / float(len(self.frame))) * y) - self.vfov/2
                vx = float(sin(thetax) * depth)
                vy = float(sin(thetay) * depth)
                vz = abs(float(cos(thetay) * depth))
                vector3 = [vx, vy, vz]
                pointArray.append(vector3)
        self.points = pointArray

    def toPCD(self):
        outputList = []
        pointCount = len(self.frame) * len(self.frame[0])
        outputList.append('# .PCD v.7 - Point Cloud Data file format')
        outputList.append('VERSION .7')
        outputList.append('FIELDS x y z')
        outputList.append('SIZE 4 4 4')
        outputList.append('TYPE F F F')
        outputList.append(f'WIDTH {pointCount}')
        outputList.append("HEIGHT 1")
        outputList.append('VIEWPOINT 0 0 0 1 0 0 0')
        outputList.append(f'POINTS {pointCount}')
        outputList.append('DATA ascii')
        for p in self.points:
            outputList.append(' '.join(map(str, p)))
        return '\n'.join(outputList)
    
    def toPointCloud2(self):
        print("do work")
