import os
import cv2

class ImageSaver:
    def __init__(self):
        pass

    def save_imgae(self, image, num):
        file_name = 'image' + str(num)
        file_path = '../image'
        if os.path.exists(file_path + '/' + file_name):
            while os.path.exists(file_path + '/' + file_name):
                num += 1
                file_name = 'image' + str(num)
            cv2.imwrite(file_path + '/' + file_name, image)
            return num
        
        