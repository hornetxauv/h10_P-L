# Annotation of datasets for Perception's Object Detection ML model

## Introduction


We'll be using CVAT for annotation. It's a useful tool with many features, of which the most important for us is auto-annotation and auto-tracking. We'll use these features and H9's pretrained model to speed up the annotation process.

There's 2 main options: online or offline. Online is much easier to set up, but the transT auto-tracking feature is limited. It's also quite slow, taking around 5-10 seconds to track 4 objects every frame.

Offline is harder to set up but apparently the transT auto-tracking is unlimited if you're running it locally. Also, if you got a beefy computer, the transT tracking can be much faster than online.