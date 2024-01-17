import cv2
import matplotlib
from os.path import join
import torch
import numpy as np
edges = [
    (9, 8), (8, 13), (13, 14), (14, 15), (8, 12), (12, 11),
    (11, 10), (8, 6), (6, 3), (3, 4), (4, 5), (6, 2),
    (2, 1), (1, 0)
]

def draw_keypoints(outputs, image, args, num_pics):
    # the `outputs` is list which in-turn contains the dictionaries 
    # for i in range(len(outputs[0])):
    outputs = (outputs + 112).cpu().detach().numpy() # 32
    D = len(outputs)
    print(D)
    x = np.zeros((1, D//2))
    y = x.copy()
    idx = 0
    for i in range(D):
        if i % 2 == 0:
            x[0, idx] = outputs[i]
        else:
            y[0, idx] = outputs[i]
            idx += 1
    keypoints = np.concatenate((y, x), axis=0)
    image = image.permute(1, 2, 0).cpu().numpy()
    image = np.ascontiguousarray(image)
    # proceed to draw the lines if the confidence score is above 0.9
    # if outputs[0]['scores'][i] > 0.9:
    # keypoints = keypoints[:, :].reshape(-1, 3)
    for p in range(keypoints.shape[1]):
        # draw the keypoints
        cv2.circle(image, (int(keypoints[1, p]), int(keypoints[0, p])), 
                    1, (0, 0, 255), thickness=-1, lineType=cv2.FILLED)
            # uncomment the following lines if you want to put keypoint number
            # cv2.putText(image, f"{p}", (int(keypoints[p, 0]+10), int(keypoints[p, 1]-5)),
            #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
    for p1, p2 in edges:
        # get different colors for the edges
        # rgb = matplotlib.colors.hsv_to_rgb([
        #     ie/float(len(edges)), 1.0, 1.0
        # ])
        # rgb = rgb*255
        # join the keypoint pairs to draw the skeletal structure
        # print(p1, p2)
        cv2.line(image, (int(keypoints[1, p1]), int(keypoints[0, p1])), (int(keypoints[1, p2]), int(keypoints[0, p2])), (0, 0, 255), 2, lineType=cv2.LINE_AA)
    # else:
    #     continue
    save_path = join(args.save_dir, args.split + '_' + str(num_pics)+'.jpg')
    cv2.imwrite(save_path, image)

