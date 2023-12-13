import os
import cv2
import time
import torch
from utils.tool import *
from module.detector import Detector

if __name__ == '__main__':
    device = "cpu"
    weight = "weights/weight_AP05:0.253207_280-epoch.pth"
    class_names = "config/coco.names"
    thresh = 0.65
    img = "test_data/0.jpg"

    # load model
    model = Detector(80, True).to(device)
    model.load_state_dict(torch.load(weight, map_location=device))
    #sets the module in eval node
    model.eval()
    
    # image preprocessing
    ori_img = cv2.imread(img)
    res_img = cv2.resize(ori_img, (352, 352), interpolation = cv2.INTER_LINEAR) 
    img = res_img.reshape(1, 352, 352, 3)
    img = torch.from_numpy(img.transpose(0, 3, 1, 2))
    img = img.to(device).float() / 255.0


    # inference
    start = time.perf_counter()
    preds = model(img)
    end = time.perf_counter()
    time = (end - start) * 1000.
    print("forward time:%fms"%time)

    output = handle_preds(preds, device, thresh)

    LABEL_NAMES = []
    with open(class_names, 'r') as f:
	    for line in f.readlines():
	        LABEL_NAMES.append(line.strip())
    
    H, W, _ = ori_img.shape
    scale_h, scale_w = H / 352., W / 352.

    for box in output[0]:
        print(box)
        box = box.tolist()
       
        obj_score = box[4]
        category = LABEL_NAMES[int(box[5])]

        x1, y1 = int(box[0] * W), int(box[1] * H)
        x2, y2 = int(box[2] * W), int(box[3] * H)

        cv2.rectangle(ori_img, (x1, y1), (x2, y2), (255, 255, 0), 2)
        cv2.putText(ori_img, '%.2f' % obj_score, (x1, y1 - 5), 0, 0.7, (0, 255, 0), 2)	
        cv2.putText(ori_img, category, (x1, y1 - 25), 0, 0.7, (0, 255, 0), 2)

    cv2.imwrite("result.png", ori_img)
