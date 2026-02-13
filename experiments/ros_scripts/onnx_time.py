#!/usr/bin/env python

import onnxruntime as ort
import numpy as np
import time

onnx_path = "/home/xtark/tarkbot/ros_ws/src/tarkbot_pure_pursuit/scripts/sac_actor_0.onnx"

if __name__ == '__main__':

    ort_sess = ort.InferenceSession(onnx_path, providers=['AzureExecutionProvider', 'CPUExecutionProvider'])
    time_array = np.zeros(100)
    
    for i in range(100):
        obs = np.random.random((1,5)).astype(np.float32)
    
        start_time = time.time()
        action = ort_sess.run(None, {"input": obs})
        end_time = time.time()
    
        time_array[i] = end_time - start_time
    print(f"{np.mean(time_array)} s")
