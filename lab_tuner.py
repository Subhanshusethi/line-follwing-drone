import cv2
import numpy as np

frameWidth = 480
frameHeight = 360

def empty(a):
    pass

def sharpen_image(image):
    # Create a sharpening kernel
    sharpen_filter = np.array([[-1, -1, -1],
                               [-1, 9, -1],
                               [-1, -1, -1]])
    # Apply the sharpening filter to the image
    sharpened_image = cv2.filter2D(image, -1, sharpen_filter)
    return sharpened_image

cv2.namedWindow("LAB")
cv2.createTrackbar("L Min", "LAB", 0, 255, empty)
cv2.createTrackbar("L Max", "LAB", 255, 255, empty)
cv2.createTrackbar("A Min", "LAB", 0, 255, empty)
cv2.createTrackbar("A Max", "LAB", 255, 255, empty)
cv2.createTrackbar("B Min", "LAB", 0, 255, empty)
cv2.createTrackbar("B Max", "LAB", 255, 255, empty)

# Load the image
img = cv2.imread("/home/subhanshu/Downloads/Photo-1(2).jpeg")

# Resize the image
img = cv2.resize(img, (frameWidth, frameHeight))
zoom_factor = 0.5

# Calculate the new dimensions after zooming
new_height = int(frameHeight * zoom_factor)
new_width = int(frameWidth * zoom_factor)

while True:
    zoomed_image = cv2.resize(img, (new_width, new_height))

    # Apply image sharpening
    sharpened_image = sharpen_image(zoomed_image)

    imgLab = cv2.cvtColor(sharpened_image, cv2.COLOR_BGR2LAB)

    l_min = cv2.getTrackbarPos("L Min", "LAB")
    l_max = cv2.getTrackbarPos("L Max", "LAB")
    a_min = cv2.getTrackbarPos("A Min", "LAB")
    a_max = cv2.getTrackbarPos("A Max", "LAB")
    b_min = cv2.getTrackbarPos("B Min", "LAB")
    b_max = cv2.getTrackbarPos("B Max", "LAB")

    lower = np.array([l_min, a_min, b_min])
    upper = np.array([l_max, a_max, b_max])

    mask = cv2.inRange(imgLab, lower, upper)
    result = cv2.bitwise_and(sharpened_image, sharpened_image, mask=mask)

    print(f'[{l_min},{a_min},{b_min},{l_max},{a_max},{b_max}]')

    mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

    hStack = np.hstack([zoomed_image, mask, result])
    cv2.imshow('Horizontal Stacking', hStack)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
