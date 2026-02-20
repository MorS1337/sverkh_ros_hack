import os, glob, cv2
from ultralytics import YOLO

PHOTOS_DIR = "photos"
RESULTS_DIR = "results"
os.makedirs(RESULTS_DIR, exist_ok=True)

TARGET_CLASSES = {"orange", "teddy bear"}
CONF = 0.4

model = YOLO("yolov8n.pt")

imgs = sorted(glob.glob(os.path.join(PHOTOS_DIR, "*.jpg")))
print("Photos:", len(imgs))

for p in imgs:
    r = model.predict(source=p, conf=CONF, verbose=False)[0]
    found = set()
    for b in r.boxes:
        cls = model.names[int(b.cls[0])]
        if cls in TARGET_CLASSES:
            found.add(cls)

    out = r.plot()
    out_path = os.path.join(RESULTS_DIR, "DETECTED_" + os.path.basename(p))
    cv2.imwrite(out_path, out)

    if found:
        print(os.path.basename(p), "->", ", ".join(sorted(found)))
    else:
        print(os.path.basename(p), "-> empty")

print("Saved to", RESULTS_DIR)