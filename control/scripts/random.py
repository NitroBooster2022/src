import cv2
import numpy as np

class LaneDetector():
    def __init__(self):
        self.maskh = np.zeros((480,640),dtype='uint8')
        h=int(0.8*480)
        polyh = np.array([[(0,h),(640,h),(640,480),(0,480)]]) # polyh might need adjustment
        cv2.fillPoly(self.maskh,polyh,255)
        self.masks = np.zeros((480,640),dtype='uint8')
        polys = np.array([[(0,300),(640,300),(640,340),(0,340)]]) # polys might need adjustment
        cv2.fillPoly(self.masks,polys,255)
        self.image = np.zeros((480,640))
        self.stopline = False
        self.dotted = False
        self.pl = 320 # previous lane center
        self.maskd = np.zeros((480,640),dtype='uint8')
        polyd = np.array([[(0,240),(0,480),(256,480),(256,240)]]) # polyd might need adjustment
        cv2.fillPoly(self.maskd,polyd,255)

    def optimized_histogram(self, image, show=False):
            self.stopline = False
            img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            h, w = 480, 640
            img_roi = cv2.bitwise_and(img_gray, self.maskh)
            threshold_value = np.clip(np.max(img_roi) - 55, 30, 200)
            _, thresh = cv2.threshold(img_roi, threshold_value, 255, cv2.THRESH_BINARY)
            hist = np.sum(thresh, axis=0)

            img_rois = cv2.bitwise_and(img_gray, self.masks)
            threshold_value_stop = np.clip(np.max(img_roi) - 65, 30, 200)
            _, threshs = cv2.threshold(img_rois, threshold_value_stop, 255, cv2.THRESH_BINARY)
            hists = np.sum(threshs, axis=0)

            def extract_lanes(hist_data):
                lane_indices = []
                previous_value = 0
                for idx, value in enumerate(hist_data):
                    if value >= 1500 and previous_value == 0:
                        lane_indices.append(idx)
                        previous_value = 255
                    elif value == 0 and previous_value == 255:
                        lane_indices.append(idx)
                        previous_value = 0
                if len(lane_indices) % 2 == 1:
                    lane_indices.append(w - 1)
                return lane_indices

            stop_lanes = extract_lanes(hists)
            for i in range(len(stop_lanes) // 2):
                if abs(stop_lanes[2 * i] - stop_lanes[2 * i + 1]) > 370 and threshold_value > 30:
                    self.stopline = True

            lanes = extract_lanes(hist)
            centers = [(lanes[2 * i] + lanes[2 * i + 1]) / 2 for i in range(len(lanes) // 2) if
                    3 < abs(lanes[2 * i] - lanes[2 * i + 1]) < 100]

            if len(centers) == 0:
                center = w / 2
            elif len(centers) == 1:
                center = (centers[0] - 0) / 2 if centers[0] > w / 2 else (centers[0] * 2 + w) / 2
            elif abs(centers[0] - centers[-1]) < 200:
                center = ((centers[0] + centers[-1]) / 2 + 0) / 2 if (centers[0] + centers[-1]) > w else (
                            centers[0] + centers[-1]) + w / 2
            else:
                center = (centers[0] + centers[-1]) / 2

            if show:
                if self.stopline:
                    cv2.putText(thresh, 'Stopline detected!', (int(w * 0.1), int(h * 0.1)), cv2.FONT_HERSHEY_SIMPLEX, 1,
                                (255, 255, 255), 1, cv2.LINE_AA)
                if self.dotted:
                    cv2.putText(image, 'DottedLine!', (int(w * 0.1), int(h * 0.3)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 1, cv2.LINE_AA)

                cv2.line(image, (int(center), int(image.shape[0])), (int(center), int(0.8 * image.shape[0])), (0, 0, 255), 5)
                add = cv2.cvtColor(thresh, cv2.COLOR_GRAY2RGB)
                cv2.imshow('Lane', cv2.add(image, add))
                cv2.waitKey(1)
            return center