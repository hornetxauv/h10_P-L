# Annotation of datasets for Perception's Object Detection ML model
## Introduction
Perception will be training and using YOLOv8's object detection model in order to recognise the targets and obstacles during the SAUVC tasks. 

(We'll be using bounding box annotation rather than other methods like pixel-tight/polygon as the objects we're dealing with are relatively simple shapes, and the background is also relatively plain, so bounding boxes would suffice for accuracy and are much faster to work with ~Nathan's unbacked personal note)

H9 has already trained a model which works pretty well on the data that we currently have, so the next step is to find/record more varied data (with different lighting, combination of objects, etc.) for training a more robust model.

In addition, as per @leftoverrice's suggestion, we'll be adding 3 additional labels to their existing model: gate-side-red, gate-side-green, and gate-top. This is to prevent the gate sides from being falsely detected as flares, which may cause unexpected behaviour.

To start with, we can first annotate a [public dataset](https://universe.roboflow.com/icbp/icbj/images/O8yB8AXId46tPpAgrQRt?) on [Roboflow](https://roboflow.com) by ICBJ (presumably one of the other participating schools). Then, we can also go back through H9's bags to add the gate sides and top to their existing annotatings (in the *_onnx.zip files), before retraining the model (perhaps from scratch).

Then, once our own vehicle is ready, we can record more footage to further improve the model.

## Method
We'll be using [CVAT](https://app.cvat.ai) for annotation. It's a useful tool with many features, of which the most important for us is auto-annotation and auto-tracking. We'll use these features and H9's pretrained model to speed up the annotation process.

There's 2 main options: online or offline. Online is much easier to set up, but the transT auto-tracking feature is limited. It's also quite slow, taking around 5-10 seconds to track 4 objects every frame. There is a built-in alternative called TrackerMIL, which is a lot faster (nearly instant) but far less accurate, which is kinda pointless if you end up having to manually adjust every bounding box in the end.

Offline is harder to set up but the transT auto-tracking has unlimited uses if you manage to get it working. Also, if you've got a beefy computer, the transT tracking can be much faster than online.

As such, it would be advisable to try setting up offline first, and only use the online app as a last resort. If it comes to that, it might be better to spend your time elsewhere.