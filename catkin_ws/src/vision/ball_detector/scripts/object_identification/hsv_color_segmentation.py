import cv2

class HsvColorSegmentation:

    def __init__(self, lower_hsv,  upper_hsv):
        self.lower_hsv = lower_hsv
        self.upper_hsv = upper_hsv
    
    def __calculate_hsv_img(self, bgr_img):
        return cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)

    def __calculate_bin_img(self, hsv_img):
        return cv2.inRange(hsv_img, self.lower_hsv, self.upper_hsv)

    def __morphology_open(self, bin_img):
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))
        return cv2.morphologyEx(bin_img, cv2.MORPH_OPEN, kernel)
        
    def __calc_centroid(self, bin_img):
        nonzero_elements = cv2.findNonZero(bin_img)
        centroid = cv2.mean(nonzero_elements)
        return (int(centroid[0]),int(centroid[1]))

    def get_obj_centroid(self, bgr_img):
        hsv_img = self.__calculate_hsv_img(bgr_img)
        bin_img = self.__calculate_bin_img(hsv_img)
        clean_bin_img = self.__morphology_open(bin_img)
        return self.__calc_centroid(clean_bin_img)

    