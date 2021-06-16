import cv2

# threshold to detect object
thresh = 0.45

cap = cv2.VideoCapture(1)
cap.set(3, 1280)
cap.set(4, 720)
cap.set(10, 70)

class_names = []
class_file = 'images.names'
with open(class_file, 'rt') as f:
    class_names = f.read().rstrip('\n').split('\n')

config_path = 'ssd_mobilenet_v3_large_images.pbtxt'
weights_path = 'frozen_interface_graph.pb'

net = cv2.dnn_DetectionModel(weights_path, config_path)
net.setInputSize((320, 320))
net.setInputScale((1.0, 127.5))
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)

while True:
    success, img = cap.read()
    class_ids, configs, bbox = net.detect(img, confThreshold=thresh)
    print(class_ids, bbox)

    if len(class_ids) != 0:
        for class_id, confidence, box in zip(class_ids.flatten(), configs.flatten(), bbox):
            cv2.rectangle(img, box, color=(0, 255, 0), thickness=2)
            cv2.putText(img, class_names[class_id - 1].upper(), box[0] + 10, box[1] + 30), \
            cv2.FONT_HERSHEY_COMPLEX, (1, (0, 255, 0), 2)
            cv2.putText(img, str(round(confidence*100, 2)), (box[0] + 200, box[1] + 30),
                        cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)

    cv2.imshow('Output', img)
    cv2.waitKey(1)
