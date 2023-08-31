import os
import torch
import cv2
import torchvision.transforms as transforms
import glob
import copy

# os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"


normalize = transforms.Normalize(
    mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]
)

transform = transforms.Compose([
    transforms.ToTensor(),
    normalize,
])

intakeSide = "switch_right"

runningNeuralNetwork = True

detections = 0

hFOV = 63
horizontalPixels = 640
verticalPixels = 480
degreesPerPixel = hFOV/horizontalPixels


device = torch.device('cuda:0')
half = device.type != 'cpu'

piLitModel = torch.load(os.path.expanduser(
    "~/mirv_ws/src/MIRV-Robot/mirv_real/Camera/weights/pi_lit_model_9.pth"))
piLitModel.eval()
piLitModel.to(device)


print("loaded model")


def detect(img, path):
    return piLitDetect(img, path)


def piLitDetect(img, orig, path):
    frame = img
    frame_orig = orig
    img = transform(img).to(device)
    if img.ndimension() == 3:
        img = img.unsqueeze(0)

    mod_out = piLitModel(img)
    piLitPrediction = mod_out[0]
    bboxList = []
    # print("DETECTING...")
    closest_track_location = None
    closest_track_distance = 5
    for bbox, score in zip(piLitPrediction["boxes"], piLitPrediction["scores"]):
        # print(score, bbox)
        if(score > 0.85 or True):
            # print("GOT A PI LIT")
            x0, y0, x1, y1 = bbox

            if x0 == 0 or x1 == horizontalPixels or y0 == 0 or y1 == verticalPixels:
                print("Ignoring detection at edge")
                continue

            if abs(x1 - x0) < 30 or abs(y1 - y0) < 8:  # abs((x1 - x0) * (y1 - y0)) < 200
                print("Ignoring small detection")
                continue

            centerX = int((x0 + x1)/2)
            centerY = int((y0 + y1)/2)
            bboxList.append(bbox)

            color = (0, 255, 0)
            if score > 0.85:
                color = (0, 255, 0)
            elif score > 0.5:
                color = (0, 255, 255)
            else:
                color = (0, 0, 255)

            frame_orig = cv2.rectangle(frame_orig, (int(x0), int(y0)),
                                       (int(x1), int(y1)), color, 3)

            frame = cv2.rectangle(frame, (int(x0), int(y0)),
                                  (int(x1), int(y1)), color, 3)

    name_suffix = path.split('\\')[-1]
    cv2.imshow(name_suffix, frame)
    return bboxList


out_dir = 'model_2_filtered_2'
images = glob.glob('./pilit_pictures/*.png')
i = 0
found = 0

if __name__ == '__main__':
    for path in images:
        try:
            print(path)
            img = cv2.imread(path)
            img = cv2.resize(img, (640, 480), interpolation=cv2.INTER_AREA)

            orig = copy.deepcopy(img)

            pi_lits = piLitDetect(img, orig, path)
            print(pi_lits)

            cv2.waitKey()

            i += 1
        except:
            pass
