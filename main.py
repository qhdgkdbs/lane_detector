import cv2
import sys
import math
import cv2 as cv
import numpy as np

cap = cv2.VideoCapture("./video_1.mp4")

INF = 9999
width = 639
height = 359
centerX = 319

polygons = np.array([
        [(0, height-100), (width, height-100), (width-199, height-210), (199, height-210)]
    ])

lower = np.array([0, 0, 200])
upper = np.array([0, 0, 255])

def canny(image):
    blur = cv2.GaussianBlur(image,(5,5),0)
    canny = cv2.Canny(blur,50,150)
    return canny

def region_of_interest(image, color):
    mask = np.zeros_like(image)
    cv2.fillPoly(mask, polygons, color)
    masked_image = cv2.bitwise_and(image, mask)
    return masked_image

def fillter(image):
    mask = cv2.inRange(image, lower, upper)
    out = cv2.bitwise_and(image, image, mask = mask)
    return out

while True:
    ret, src = cap.read()
    src = cv2.resize(src, (width+1, height+1))

    dst = cv.Canny(src, 50, 200, None, 3)
    masked_dst = region_of_interest(dst, (255, 255, 255))
    cdstP = cv.cvtColor(masked_dst, cv.COLOR_GRAY2BGR)

    masked_cdstP = region_of_interest(cdstP, (255, 255, 255))
    masked_src = region_of_interest(src, (255, 255, 255))

    linesP = cv.HoughLinesP(masked_dst, 1, np.pi / 180, 50, None, 50, 10)

    if linesP is not None:
        for i in range(0, len(linesP)):
            l = linesP[i][0]

            # 각도가 30도 이하면 걸러내기

            degree = math.tan(math.radians(30))
            if math.atan(degree) > 30
            cv.line(masked_cdstP, (l[0], l[1]), (l[2], l[3]), (0, 0, 255), 10, cv.LINE_AA)

    fm_cdstP = fillter(masked_cdstP)

    ############
    # 여기서 연산 #
    ############
    cv.line(src, (centerX, 0), (centerX, 359), (0, 255, 0), 1, cv.LINE_AA)
    cv.line(fm_cdstP, (centerX, 0), (centerX, 359), (0, 255, 0), 1, cv.LINE_AA)

    left_red_x = []
    left_red_y = []

    # 좌측 차선 탐지
    count = 0
    for y in range(height-72, int((height+1)/2), -13):
        for x in range(centerX, 0, -3):
            if fm_cdstP[y][x][2] > 240:
                left_red_x.append(x)
                left_red_y.append(y)
                count = count + 1
                break
        if count >= 2:
            break

    # 우측 차선 탐지
    right_red_x = []
    right_red_y = []

    count = 0
    for y in range(height-72, int((height+1)/2), -13):
        for x in range(centerX, width, 3):
            if fm_cdstP[y][x][2] > 240:
                right_red_x.append(x)
                right_red_y.append(y)
                count = count + 1
                break
        if count >= 2:
            break

    lGrad = INF
    if (len(left_red_x) >= 2):
        # left절편의 기울기를 구할 수 있는 경우
        if left_red_x[1] != left_red_x[0]:
            #같은 좌표가 아닐 경우
            lGrad = (left_red_y[1] - left_red_y[0]) / (left_red_x[1] - left_red_x[0])
        else:
            print("left 절편의 X좌표가 같습니다 ")
    else:
        # left절편에 기울기가 없는 경우.
        print("no line on left")

    rGrad = INF
    if (len(right_red_x) >= 2):
        # right절편의 기울기를 구할 수 있는 경우
        if right_red_x[1] != right_red_x[0]:
            #같은 좌표가 아닐 경우
            rGrad = (right_red_y[1] - right_red_y[0]) / (right_red_x[1] - right_red_x[0])
        else:
            print("right절편의 x좌표가 같습니다.")
    else:
        #right절편의 기울기가 없는경우
        print("no line on right")


    if (lGrad!=INF) and (rGrad!=INF):
        #기울기가 무한대가 아닐경우 (정상일 가능성이 높을 경)
        b0 = right_red_y[0] - lGrad * right_red_x[0]
        b1 = right_red_y[1] - rGrad * right_red_x[1]
        cPoint = []

        if lGrad == rGrad:
            # 오른쪽 혹은 왼쪽선이 중앙선을 넘었을떄,하나의 선을 두곳에서 인식했을 경우.
            if left_red_y[0] > right_red_y[0]:
                print("(기울기는 같은데, 오른쪽 절편에서 시작된 직선)left")
            else:
                print("(기울기는 같은데, 왼쪽 절편에서 시작된 직선)right")
        else:
            #기울기가 서로 다를 경우
            cPoint.append((b1 - b0) / (lGrad - rGrad))
            # cPoint.append(lGrad * (cPoint[0] + b0))

        # 교점이랑 초록선 비교
        # 교점이 있을 경우(양쪽에 기울기 존재)
        if(len(cPoint)):
            if cPoint[0] < (centerX):
                print("left")
                cv.line(fm_cdstP, (centerX - 200, 0), (centerX - 250, 0), (255, 0, 0), 80, cv.LINE_AA)
            elif cPoint[0] > (centerX):
                print("right")
                cv.line(fm_cdstP, (centerX + 200, 0), (centerX + 250, 0), (255, 0, 0), 80, cv.LINE_AA)
            else:
                print("center")
    # 기울기 중 무한대가 있을경우
    elif (lGrad!=INF) and (rGrad==INF):
        print("INFright")
        cv.line(fm_cdstP, (centerX + 200, 0), (centerX + 250, 0), (255, 0, 0), 100, cv.LINE_AA)
    elif (lGrad == INF) and (rGrad != INF):
        print("INFleft")
        cv.line(fm_cdstP, (centerX - 200, 0), (centerX - 250, 0), (255, 0, 0), 100, cv.LINE_AA)

    else:
        print("center")

    # 출력
    cv.imshow("Source", src)
    cv.imshow("Masked Source", canny(src))
    cv.imshow("Detected Lines (in red) - Probabilistic Line Transform", fm_cdstP)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
