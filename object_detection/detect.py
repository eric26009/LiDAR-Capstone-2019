from imageai.Detection import ObjectDetection
import time
import numpy as np
import os
import cv2


file_name = 'person_point_cloud.jpg'
img = cv2.imread(file_name)
grayscale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
smoothed = cv2.GaussianBlur(grayscale,(3,3),0)
filter_image_name = file_name[0:len(file_name)-4]+"_smoothed.jpg"
cv2.imwrite(filter_image_name, smoothed)

execution_path = os.getcwd()
detector = ObjectDetection()
detector.setModelTypeAsYOLOv3()
detector.setModelPath( os.path.join(execution_path , "yolo.h5"))
detector.loadModel()
#custom_objects = detector.CustomObjects(person=True)
detections = detector.detectCustomObjectsFromImage(input_image=os.path.join(execution_path , filter_image_name), output_image_path=os.path.join(execution_path ,filter_image_name[0:len(filter_image_name)-4]+"_detected.jpg"), minimum_percentage_probability=20)


for eachObject in detections:
	print(eachObject["name"] + " : " + str(eachObject["percentage_probability"]) )
	print("--------------------------------")
