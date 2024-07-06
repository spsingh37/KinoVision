import cv2
import numpy as np

def label_blocks( depth_data, cnt_image, lower=785, upper=815, full = False):
    #color_image, depth_data = allign_depth_to_color()
    
    # working spacce
    if full: 
        lower_right_pos = (1120,650)
    else:
        lower_right_pos = (1120,509)

    lower_right_pos = (1120,650)

    # apply a bilateral filter (kernel=5) to smooth depth data
    depth_data = depth_data.astype(np.float32)
    depth_data = cv2.bilateralFilter(depth_data, 11, 100, 100)


    # correct depth data
    for y in range(720):
        depth_data[y] = depth_data[y] - 0.2292 * (y-98)


    # apply a mask to obtain working space
    mask = np.zeros_like(depth_data, dtype=np.uint8)
    cv2.rectangle(mask, (283,109),lower_right_pos, 255, cv2.FILLED)
    cv2.rectangle(mask, (633,412),(769,659), 0, cv2.FILLED)
    cv2.rectangle(cnt_image, (283,109), lower_right_pos, (0, 0, 255), 2)
    cv2.rectangle(cnt_image, (633,412),(769,659), (0, 0, 255), 2)
    thresh = cv2.bitwise_and(cv2.inRange(depth_data, lower, upper), mask)

    # apply erosion and dilation to denoise
    kernel = np.ones((5, 5), np.uint8)
    thresh = cv2.erode(thresh, kernel, iterations=3) 
    thresh = cv2.dilate(thresh, kernel, iterations=3)


    # find countours in the binary threshold image
    _, contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(cnt_image, contours, -1, (0,255,255), thickness=1)




    return thresh