[basic]
type = axmodel
model_npu  = steelball_yolo11n_v4_leftedge_best_320x416_maixcam2_npu.axmodel
model_vnpu = steelball_yolo11n_v4_leftedge_best_320x416_maixcam2_vnpu.axmodel

[extra]
model_type = yolo11
type = detector
input_type = rgb
labels = ball
input_cache = true
output_cache = true
input_cache_flush = false
output_cache_inval = true
mean = 0,0,0
scale = 0.00392156862745098, 0.00392156862745098, 0.00392156862745098