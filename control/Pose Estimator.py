


import cv2

import time

from PoseDetector.PoseModule import PoseDetector  #posedetector module

vidpath = r'C:\Users\Abhishek Raman\Desktop\Abhishek\Image Processing stuff\Roboticarm.mp4'


cap = cv2.VideoCapture(vidpath)

pTime = 0

detector = PoseDetector()

while True:

    success, img = cap.read()

    img = detector.findPose(img)

    lmList = detector.getPosition(img)

    print(lmList)



    cTime = time.time()

    fps = 1 / (cTime - pTime)

    pTime = cTime



    cv2.putText(img, str(int(fps)), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)

    cv2.imshow("Image", img)

    cv2.waitKey(1)