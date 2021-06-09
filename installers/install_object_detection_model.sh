#!/usr/bin/env bash
echo " =============================================="
echo "| Downloading the ssd_mobilenet_v2_coco model  |"
echo " =============================================="
mkdir -p /opt/models
python3 /opt/intel/openvino_2021/deployment_tools/tools/model_downloader/downloader.py --name ssd_mobilenet_v2_coco -o /opt/models
echo ""
echo " ============================================="
echo "| Optimizing the ssd_mobilenet_v2_coco model  |"
echo " ============================================="
python3 /opt/intel/openvino_2021/deployment_tools/model_optimizer/mo_tf.py --input_model /opt/models/public/ssd_mobilenet_v2_coco/ssd_mobilenet_v2_coco_2018_03_29/frozen_inference_graph.pb --tensorflow_object_detection_api_pipeline_config /opt/models/public/ssd_mobilenet_v2_coco/ssd_mobilenet_v2_coco_2018_03_29/pipeline.config --input_shape [1,300,300,3] --tensorflow_use_custom_operations_config /opt/intel/openvino_2021/deployment_tools/model_optimizer/extensions/front/tf/ssd_v2_support.json --output_dir /opt/models
echo ""
echo " ==================================================================="
echo "| Downloading and optimizing ssd_mobilenet_v2_coco model completed  |"
echo "| Optimized artifacts created in /opt/models directory              |"
echo " ==================================================================="
