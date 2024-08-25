#!/usr/bin/env python3
import sys
import os
import tensorflow as tf

class ModelLoader:
    _model = None

    @staticmethod
    def load_model():
        if ModelLoader._model is None:
            script_dir = os.path.dirname(__file__)
            model_dir = os.path.join(script_dir,'..', 'models', 'ssd_mobilenet_v2_fpnlite_320x320_coco17_tpu-8', 'saved_model')
            model_dir = os.path.abspath(model_dir)
            print(f"Loading model from {model_dir}")
            ModelLoader._model = tf.saved_model.load(model_dir)
        return ModelLoader._model
