from sensor import Sensor
from wavelength import WaveLengthIDs
from pointcloud import PointCloud
from datetime import datetime, date

HORIZONTAL_FOV_IN_DEG = 64
VERTICAL_FOV_IN_DEG = 36


class main:
    def __init__(self):
        self.sensor = Sensor()
        self.pointCloud: PointCloud = None
        while(1):
            self.run()
            if(input("press enter to take another snapshot, -1 to exit...") == str(-1)):
                return

    def run(self) -> None:
        cameraConnected, allDepth, allAmp, depthValues, amplitudeValues = self.sensor.getFrame(
            WaveLengthIDs['830'])

        if (not cameraConnected):
            return
        self.pointCloud = PointCloud(
            allDepth,
            HORIZONTAL_FOV_IN_DEG,
            VERTICAL_FOV_IN_DEG
        )
        self.__saveToFile(self.pointCloud.toPCD())

    def __saveToFile(self, s: str):
        filename = f'./output/snapshot_{"{:%m_%d_%H_%M_%S}".format(datetime.now())}.pcd'
        with open(filename, 'w') as f:
            f.write(s)
            print(f'saved to: {filename}')


# Execute
main()
