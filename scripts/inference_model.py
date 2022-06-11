import os
import numpy as np
import time

import matplotlib.pyplot as plt
from object_detection.utils import visualization_utils as viz_utils

import tensorflow as tf
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--model_path', '-m', dest='model_path', default=None)

args = parser.parse_args()
model_path = os.path.join(args.model_path, 'saved_model/')
category_index = {1 : {'id' : 1, 'name':'helipad'},
                  2 : {'id' : 2, 'name':'Vehicle'},
                  3 : {'id' : 3, 'name':'Person'},}


# Load model
def load_model(model_path):
    start_time = time.time()
    tf.keras.backend.clear_session()
    detect_fn = tf.saved_model.load(model_path)
    end_time = time.time()
    elapsed_time = end_time - start_time
    print('Elapsed time: ' + str(elapsed_time) + 's')
    return detect_fn


# Inference
def inference_model(detect_fn, image_np):
    input_tensor = np.expand_dims(image_np, 0)
    start_time = time.time()
    detections = detect_fn(input_tensor)
    end_time = time.time()
    # elapsed.append(end_time - start_time)

    plt.rcParams['figure.figsize'] = [42, 21]
    label_id_offset = 1
    image_np_with_detections = image_np.copy()
    viz_utils.visualize_boxes_and_labels_on_image_array(
            image_np_with_detections,
            detections['detection_boxes'][0].numpy(),
            detections['detection_classes'][0].numpy().astype(np.int32),
            detections['detection_scores'][0].numpy(),
            category_index,
            use_normalized_coordinates=True,
            max_boxes_to_draw=200,
            min_score_thresh=.40,
            agnostic_mode=False)

    return image_np, image_np_with_detections

