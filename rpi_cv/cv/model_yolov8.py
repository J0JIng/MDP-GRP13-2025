import os
import shutil
import time
import glob
import torch
from PIL import Image
import cv2
import random
import string
import numpy as np

# NEW: Ultralytics YOLO (v8 / v12)
from ultralytics import YOLO

# Ensure required folders exist (match how your Flask app writes/reads)
for d in ["uploads", "runs", os.path.join("runs", "originals"), "own_results"]:
    os.makedirs(d, exist_ok=True)

# mapping logic dictionary
NAME_TO_ID = {
    "NA": 'NA',
    "Bullseye": 10,
    "one": 11, "two": 12, "three": 13, "four": 14, "five": 15,
    "six": 16, "seven": 17, "eight": 18, "nine": 19,
    "A": 20, "B": 21, "C": 22, "D": 23, "E": 24, "F": 25, "G": 26, "H": 27,
    "S": 28, "T": 29, "U": 30, "V": 31, "W": 32, "X": 33, "Y": 34, "Z": 35,
    "up": 36, "down": 37, "right": 38, "left": 39,
    "up arrow": 36, "down arrow": 37, "right arrow": 38, "left arrow": 39,
    "circle": 40
}


def load_model(weights_path="./weights/detectionv1.pt", device=None):
    """
    Load a YOLOv8/YOLOv12 model from ultralytics.
    - weights_path: path to your .pt (e.g., 'best.pt', 'yolov8n.pt', 'Week_9.pt')
    """
    # Create model from weights
    model = YOLO(weights_path)

    # Select device if specified, else auto
    if device is None:
        device = "cuda" if torch.cuda.is_available() else "cpu"
    model.to(device)
    return model


def draw_own_bbox(img, x1, y1, x2, y2, label, color=(36, 255, 12), text_color=(0, 0, 0)):
    """
    Draw bbox + label and save both raw and annotated images in own_results/
    (kept from your original code)
    """

    label = f"{label}-{NAME_TO_ID.get(label, 'NA')}"
    x1, x2, y1, y2 = int(x1), int(x2), int(y1), int(y2)
    rand = str(int(time.time()))

    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    cv2.imwrite(f"own_results/raw_image_{label}_{rand}.jpg", img_rgb)

    img_rgb = cv2.rectangle(img_rgb, (x1, y1), (x2, y2), color, 2)
    (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
    img_rgb = cv2.rectangle(img_rgb, (x1, y1 - 20), (x1 + w, y1), color, -1)
    img_rgb = cv2.putText(img_rgb, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, text_color, 1)
    cv2.imwrite(f"own_results/annotated_image_{label}_{rand}.jpg", img_rgb)


def _results_to_dicts_v8(result):
    """
    Convert a single Ultralytics result (v8+) to list[dict] like v5's pandas() output.
    Each dict has: xmin, ymin, xmax, ymax, confidence, name, bboxArea
    """
    boxes = result.boxes  # Boxes object
    if boxes is None or boxes.data is None or len(boxes) == 0:
        return []

    xyxy = boxes.xyxy.cpu().numpy()      # (N,4)
    conf = boxes.conf.cpu().numpy()      # (N,)
    cls = boxes.cls.cpu().numpy().astype(int)  # (N,)
    names = result.names  # dict: class_id -> name

    dicts = []
    for i in range(len(xyxy)):
        x1, y1, x2, y2 = xyxy[i]
        c = float(conf[i])
        cls_id = int(cls[i])
        name = names.get(cls_id, str(cls_id))
        bboxHt = float(y2 - y1)
        bboxWt = float(x2 - x1)
        dicts.append({
            "xmin": float(x1), "ymin": float(y1),
            "xmax": float(x2), "ymax": float(y2),
            "confidence": c, "name": name,
            "bboxHt": bboxHt, "bboxWt": bboxWt,
            "bboxArea": bboxHt * bboxWt
        })
    # sort by area descending (to match your prior behavior)
    dicts.sort(key=lambda d: d["bboxArea"], reverse=True)
    print(dicts)
    return dicts


def _save_v8_visual(result, save_dir="runs"):
    """
    Save YOLOv8 annotated image into runs/ (near your previous v5 behavior).
    """
    os.makedirs(save_dir, exist_ok=True)
    plotted = result.plot()  # returns a BGR numpy image with annotations
    # unique filename
    out_path = os.path.join(save_dir, f"detect_{int(time.time())}.jpg")
    cv2.imwrite(out_path, plotted)
    return out_path

# accounts for single bulleyes case
def predict_image(image, model, signal): 
    """
    Original Week 8 logic adapted to v8+.
    Keeps filename-based logic you already had: area sorting + signal L/R/C. 
    """
    try:
        img_path = os.path.join('uploads', image)
        img_pil = Image.open(img_path)
        results = model.predict(img_pil, verbose=False)  # list of Results
        r = results[0]
        _save_v8_visual(r, save_dir="runs")  # mimic v5 results.save('runs')

        preds_all = _results_to_dicts_v8(r) 
        pred = "NA"  # initialized to NA for final prediction

        # --- Minimal addition: handle single Bullseye case ---
        if len(preds_all) == 1 and preds_all[0]["name"] == "Bullseye":
            pred = preds_all[0]
        # -----------------------------------------------------

        # Filter out Bullseye by name first
        else:
            preds = [d for d in preds_all if d["name"] != "Bullseye"]

            # Detected 1 object
            if len(preds) == 1:
                pred = preds[0]

            # Detected multiple objects
            elif len(preds) > 1:
                pred_shortlist = []
                current_area = preds[0]["bboxArea"]
                for d in preds:
                    if d["name"] != "Bullseye" and d["confidence"] > 0.5 and (
                        d["bboxArea"] >= current_area * 0.8 or
                        (d["name"] == "one" and d["bboxArea"] >= current_area * 0.6)
                    ):
                        pred_shortlist.append(d)
                        current_area = d["bboxArea"]

                if len(pred_shortlist) == 1:
                    pred = pred_shortlist[0]
                elif len(pred_shortlist) > 1:
                    pred_shortlist.sort(key=lambda d: d["xmin"])  # left-to-right
                    if signal == 'L':
                        pred = pred_shortlist[0]
                    elif signal == 'R':
                        pred = pred_shortlist[-1]
                    else:
                        chosen = None
                        for d in pred_shortlist:
                            if 250 < d["xmin"] < 774:
                                chosen = d
                                break
                        pred = chosen if chosen else sorted(pred_shortlist, key=lambda d: d["bboxArea"])[-1]


        if isinstance(pred, dict):
            draw_own_bbox(np.array(img_pil), pred['xmin'], pred['ymin'], pred['xmax'], pred['ymax'], pred['name'])
            image_id = str(NAME_TO_ID[pred['name']])
            pred_conf = float(pred["confidence"])
        else:
            image_id = "NA"
            pred_conf = 0.0

        print(f"Final result: {image_id}")
        print(f"Predicted confidence: {pred_conf:.4f}")
        return image_id, pred_conf

    except Exception as e:
        print(f"Final result: NA ({e})")
        return "NA", 0.0


def predict_image_week_9(image, model):
    """
    Your Week 9 simplified logic, now on v8+:
    - pick first non-Bullseye detection with conf > 0.5 after sorting by area. 
    """
    img_path = os.path.join('uploads', image)
    img_pil = Image.open(img_path)

    results = model.predict(img_pil, verbose=False)
    r = results[0]
    _save_v8_visual(r, save_dir="runs")

    preds = _results_to_dicts_v8(r)
    pred = 'NA'
    if len(preds) > 0:
        for d in preds:
            if d["name"] != "Bullseye" and d["confidence"] > 0.5:
                pred = d
                break

        if not isinstance(pred, str):
            draw_own_bbox(np.array(img_pil), pred['xmin'], pred['ymin'], pred['xmax'], pred['ymax'], pred['name'])

    image_id = str(NAME_TO_ID[pred['name']]) if not isinstance(pred, str) else 'NA'
    return image_id

def auto_grid(n:int):
    """
    Given n images, return (rows,cols) for stitching.
    """
    if n <= 0: return (0,0)
    if n == 1: return (1,1)
    if n == 2: return (2,1)
    if n == 3: return (3,1)
    if n == 4: return (2,2) 
    if n <= 6: return (3,2)
    return (2,4)  # max 8 images supported


def stitch_image_own():
    """
    Stitches images saved in own_results/ (kept from your original function) 
    """
    imgFolder = 'own_results'
    stitchedPath = os.path.join(imgFolder, f'stitched-{int(time.time())}.jpeg')

    imgPaths = glob.glob(os.path.join(imgFolder + "/annotated_image_*.jpg"))
    imgTimestamps = [imgPath.split("_")[-1][:-4] for imgPath in imgPaths]

    sortedByTimeStampImages = sorted(zip(imgPaths, imgTimestamps), key=lambda x: x[1])
    images = [Image.open(x[0]) for x in sortedByTimeStampImages]
    if not images:
        return Image.new('RGB', (1, 1))
    
    # Use the most recent up to 8 images (still works if fewer than 4 exist)
    take = min(8, len(sortedByTimeStampImages))
    selectedPairs = sortedByTimeStampImages[-take:]

    # Open as RGB
    images = [Image.open(p).convert('RGB') for p, _ in selectedPairs]
    n = len(images)
    rows, cols = auto_grid(n)

    # Auto cell size: derive from median-ish height (clamped), compute max width when scaled to that height
    heights = sorted([im.size[1] for im in images if im.size[1] > 0])
    if not heights:
        return Image.new('RGB', (1, 1))
    cell_h = heights[len(heights)//2]
    cell_h = max(240, min(600, cell_h))  # sane visual clamp

    scaled_ws = []
    for im in images:
        w, h = im.size
        scale = cell_h / float(h) if h > 0 else 1.0
        scaled_ws.append(int(max(1, round(w * scale))))
    cell_w = max(scaled_ws) if scaled_ws else cell_h

    pad = 10
    bg = (0, 0, 0)
    W = cols * cell_w + (cols + 1) * pad
    H = rows * cell_h + (rows + 1) * pad
    stitchedImg = Image.new('RGB', (W, H), bg)

    # Paste centered into each cell (preserve aspect)
    for idx, im in enumerate(images):
        r, c = divmod(idx, cols)
        if r >= rows: break
        x0 = pad + c * (cell_w + pad)
        y0 = pad + r * (cell_h + pad)
        w, h = im.size
        scale = min(cell_w / float(w) if w > 0 else 1.0,
                    cell_h / float(h) if h > 0 else 1.0)
        nw, nh = max(1, int(w * scale)), max(1, int(h * scale))
        im_resized = im.resize((nw, nh), Image.LANCZOS)
        ox = x0 + (cell_w - nw) // 2
        oy = y0 + (cell_h - nh) // 2
        stitchedImg.paste(im_resized, (ox, oy))
        im.close()

    stitchedImg.save(stitchedPath, quality=90)
    return stitchedImg







# # initial prediction function
# def predict_image(image, model, signal):
#     """
#     Original Week 8 logic adapted to v8+.
#     Keeps filename-based logic you already had: area sorting + signal L/R/C. 
#     """
#     try:
#         img_path = os.path.join('uploads', image)
#         img_pil = Image.open(img_path)
#         results = model.predict(img_pil, verbose=False)  # list of Results
#         r = results[0]
#         _save_v8_visual(r, save_dir="runs")  # mimic v5 results.save('runs')

#         preds = _results_to_dicts_v8(r) # list of objects detected if more than 1 object detected
#         pred = "NA" # initialzed to NA for final prediction

#         # Filter out Bullseye by name first
#         preds = [d for d in preds if d["name"] != "Bullseye"]

#         # Detected 1 object
#         if len(preds) == 1:
#             pred = preds[0]

#         # Detected multiple objects
#         elif len(preds) > 1:
#             pred_shortlist = []
#             current_area = preds[0]["bboxArea"]
#             for d in preds:
#                 if d["name"] != "Bullseye" and d["confidence"] > 0.5 and (
#                     d["bboxArea"] >= current_area * 0.8 or
#                     (d["name"] == "one" and d["bboxArea"] >= current_area * 0.6)
#                 ):
#                     pred_shortlist.append(d)
#                     current_area = d["bboxArea"]

#             if len(pred_shortlist) == 1:
#                 pred = pred_shortlist[0]
#             elif len(pred_shortlist) > 1:
#                 pred_shortlist.sort(key=lambda d: d["xmin"])  # left-to-right
#                 if signal == 'L':
#                     pred = pred_shortlist[0]
#                 elif signal == 'R':
#                     pred = pred_shortlist[-1]
#                 else:
#                     # choose central (~ your logic) else largest area
#                     chosen = None
#                     for d in pred_shortlist:
#                         if 250 < d["xmin"] < 774:
#                             chosen = d
#                             break
#                     pred = chosen if chosen else sorted(pred_shortlist, key=lambda d: d["bboxArea"])[-1]

#         # if not isinstance(pred, str):
#         #     draw_own_bbox(np.array(img_pil), pred['xmin'], pred['ymin'], pred['xmax'], pred['ymax'], pred['name'])

#         NAME_TO_ID = {
#             "NA": "NA",
#             "Bullseye": 10,
#             "one": 11, "two": 12, "three": 13, "four": 14, "five": 15,
#             "six": 16, "seven": 17, "eight": 18, "nine": 19,
#             "A": 20, "B": 21, "C": 22, "D": 23, "E": 24, "F": 25, "G": 26, "H": 27,
#             "S": 28, "T": 29, "U": 30, "V": 31, "W": 32, "X": 33, "Y": 34, "Z": 35,
#             "up": 36, "down": 37, "right": 38, "left": 39,
#             "up arrow": 36, "down arrow": 37, "right arrow": 38, "left arrow": 39,
#             "circle": 40
#         }

#         if isinstance(pred, dict):
#             draw_own_bbox(np.array(img_pil), pred['xmin'], pred['ymin'], pred['xmax'], pred['ymax'], pred['name'])
#             image_id = str(NAME_TO_ID[pred['name']]) if not isinstance(pred, str) else 'NA'
#             pred_conf = float(pred["confidence"])
#         else:
#             image_id = "NA"
#             pred_conf = 0.0
        
#         print(f"Final result: {image_id}")
#         print(f"Predicted confidence: {pred_conf:.4f}")
#         return image_id, pred_conf
#     except Exception as e:
        # print(f"Final result: NA ({e})")
        # return 'NA', 0.0