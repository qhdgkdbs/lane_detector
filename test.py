import cv2
import sys
import math
import cv2 as cv
import numpy as np

cap = cv2.VideoCapture("./video2.mp4")
polygons = np.array([
    [(0, 285), (639, 285), (400, 149), (200, 149)]
])


def canny(image):
    blur = cv2.GaussianBlur(image, (5, 5), 0)
    canny = cv2.Canny(blur, 50, 150)
    return canny


def region_of_interest(image):
    mask = np.zeros_like(image)
    cv2.fillPoly(mask, polygons, (255, 255, 255))
    masked_image = cv2.bitwise_and(image, mask)
    return masked_image


while (True):
    ret, src = cap.read()
    src = cv2.resize(src, (640, 360))

    dst = cv.Canny(src, 50, 200, None, 3)
    masked_dst = region_of_interest(dst)
    cdstP = cv.cvtColor(masked_dst, cv.COLOR_GRAY2BGR)

    masked_cdstP = region_of_interest(cdstP)
    masked_src = region_of_interest(src)

    linesP = cv.HoughLinesP(masked_dst, 1, np.pi / 180, 50, None, 50, 10)

    if linesP is not None:
        for i in range(0, len(linesP)):
            l = linesP[i][0]
            cv.line(masked_cdstP, (l[0], l[1]), (l[2], l[3]), (0, 0, 255), 3, cv.LINE_AA)

    # 여기서 연산

    cv.imshow("Source", src)
    cv.imshow("Detected Lines (in red) - Probabilistic Line Transform", masked_cdstP)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
