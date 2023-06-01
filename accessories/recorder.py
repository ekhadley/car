import numpy, time, cv2, time
from PIL import Image

vid = cv2.VideoCapture(0)

count = 0


while count > -1:
    time.sleep(0)
    count += 1

    ret, frame = numpy.array(vid.read())

    path  = 'C:\\Users\\ekhad\\Desktop\\lvid\\frame' + str(count) + '.png'
    cv2.imwrite(path, frame)

    print(path)
    cv2.imshow('frame', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'): 
        1





















































