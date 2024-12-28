# use with:
# cvat-test % cvat-cli --server-host {server host, either app.cvat.ai or localhost:8080} --auth {cvat username} task auto-annotate {task id, get it from the cvat interface} --function-file ./h9-model.py --allow-unmatched-labels --clear-existing

from ultralytics import YOLO
import cvat_sdk.auto_annotation as cvataa
from os import environ
from dotenv import load_dotenv

load_dotenv()

model_path = environ.get("MODEL_PATH")

if not model_path:
    raise "Make sure to set the path to the h9 model 'MODEL_PATH' in .env"

_model = YOLO(model_path, task="detect")

for id, name in _model.names.items():
    print(id, name)

spec = cvataa.DetectionFunctionSpec(
    labels=[cvataa.label_spec(name, id) for id, name in _model.names.items()]
)

def _yolo_to_cvat(results):
    for result in results:
        for box, label in zip(result.boxes.xyxy, result.boxes.cls):
            yield cvataa.rectangle(int(label.item()), [p.item() for p in box])

def detect(context, image):
    return list(_yolo_to_cvat(_model.predict(source=image, verbose=False, imgsz=[480, 640])))
