# use with:
# cvat-test % cvat-cli --server-host {server host, either app.cvat.ai or localhost:8080} --auth {cvat username} task auto-annotate {task id, get it from the cvat interface} --function-file ./h9-model.py --allow-unmatched-labels --clear-existing

import PIL.Image
from ultralytics import YOLO

import cvat_sdk.auto_annotation as cvataa
import cvat_sdk.models as models

model_path = '/Users/nathan/development/HornetX/ML/test-ML/models/h9.onnx'

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
