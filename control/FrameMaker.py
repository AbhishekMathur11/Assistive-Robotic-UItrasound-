import cv2
vidcap = cv2.VideoCapture(r'C:\Users\Abhishek Raman\Desktop\Abhishek\Image Processing stuff\Ultrasound_demo.mp4')
success,image = vidcap.read()
count = 0
while success:
  cv2.imwrite("frame%d.jpg" % count, image)     # save frame as JPEG file      
  success,image = vidcap.read()
  print('Read a new frame: ', success)
  count += 1