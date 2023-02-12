import apriltag
import cv2
import numpy as np

at_detector = Detector(
   families="tag16h5",
   nthreads=1,
   quad_decimate=1.0,
   quad_sigma=0.0,
   refine_edges=1,
   decode_sharpening=0.25,
   debug=0
)

cam = cv2.VideoCapture(0)
result, image = cam.read()
cv2.imshow("image", image)
newimage = np.mean(np.array(image.copy(), dtype=np.uint8), axis=2, dtype=np.uint8)

'''
for i in range(len(at_detector.detect(newimage))):
  image = cv2.rectangle(image, (int(at_detector.detect(newimage)[i].corners[0][0]), int(at_detector.detect(newimage)[i].corners[0][1])), (int(at_detector.detect(newimage)[i].corners[3][0]), int(at_detector.detect(newimage)[i].corners[3][1])), color=(0,0,255), thickness=2)

cv2.imshow("image2", image)
'''
detector = apriltag.Detector()
result = detector.detect(newimage)

print(result)

cv2.waitKey(5000)