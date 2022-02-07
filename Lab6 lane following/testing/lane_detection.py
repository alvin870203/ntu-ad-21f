import cv2
import numpy as np

img = cv2.imread("./tmp.jpg", cv2.IMREAD_COLOR)
img_org = cv2.imread("tmp.jpg", cv2.IMREAD_COLOR)

img_height, img_width = img.shape[:2]

src = np.float32([[100, 901], [1820, 901],
                  [498, 726], [1422, 726]])
dst = np.float32([[100, 1080], [1820, 1080],
                  [100, 0], [1820, 0]])



M = cv2.getPerspectiveTransform(src, dst)
img_warped = cv2.warpPerspective(img, M, [img_width, img_height])

img_hsv = cv2.cvtColor(img_warped, cv2.COLOR_BGR2HSV)
mask_yello = cv2.inRange(img_hsv, (14,127,0), (20,255,255))
mask_white = cv2.inRange(img_hsv, (0,0,127), (279,25,255))
mask_lane = cv2.bitwise_or(mask_yello, mask_white)


print(img_height, img_width)
# cv2.imshow("warped", img_warped)
cv2.imshow("warped", mask_lane)
cv2.waitKey(0)

histogram = np.sum(mask_lane[img_height // 2:, :], axis=0)
