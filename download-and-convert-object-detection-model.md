# Download and convert the object-detection model

The Follow the Leader (FTL) sample project is an sample application built on top of the existing AWS DeepRacer application, which uses an object-detection machine learning model through which the AWS DeepRacer device can identify and follow a person. For more details about the FTL sample project, see the [AWS DeepRacer Follow the Leader (FTL) sample project](https://github.com/aws-deepracer/aws-deepracer-follow-the-leader-sample-project).

The FTL sample project uses a sample open-source object-detection model suited to run high-performance inference with minimum latency on the AWS DeepRacer device. The FTL sample project is designed to make it easy to replace this model with any other custom model and run the inference on it. We have tested the object detection using the Intel OpenVino Optimized [ssd_mobilenet_v2_coco](https://docs.openvinotoolkit.org/latest/omz_models_public_ssd_mobilenet_v2_coco_ssd_mobilenet_v2_coco.html) object-detection model built on the [Single-Shot multibox Detection (SSD)](https://arxiv.org/abs/1801.04381) network as the default model.

## Download the model

As a prerequisite to using the FTL sample project, open a terminal and run the following instructions as the root user on the AWS DeepRacer device to download the [ssd_mobilenet_v2_coco](https://docs.openvinotoolkit.org/latest/omz_models_public_ssd_mobilenet_v2_coco_ssd_mobilenet_v2_coco.html) object detection model:

- Create a model directory:

        mkdir /opt/models

- Navigate to the Intel OpenVino model downloader installation directory:

        cd /opt/intel/openvino_2021/deployment_tools/tools/model_downloader

- Run the downloader script to download the model:

        python3 downloader.py --name ssd_mobilenet_v2_coco -o <target_download_path>

    Example: 

        python3 downloader.py --name ssd_mobilenet_v2_coco -o /opt/models


## Optimize the model

Run Intel Model Optimizer to optimize the [ssd_mobilenet_v2_coco](https://docs.openvinotoolkit.org/latest/omz_models_public_ssd_mobilenet_v2_coco_ssd_mobilenet_v2_coco.html) object-detection model and convert it into OpenVino Intermediate Representation. For more information about converting a Tensorflow model to OpenVINO optimized format, see [Converting a TensorFlow Model](https://docs.openvinotoolkit.org/2021.1/openvino_docs_MO_DG_prepare_model_convert_model_Convert_Model_From_TensorFlow.html).

To optimize the model, open a terminal and run the following instructions as the root user on the AWS DeepRacer device:

- Navigate to the Intel OpenVino Model Optimizer installation directory:

        cd /opt/intel/openvino_2021/deployment_tools/model_optimizer

- Run the model optimizer script with the required parameters to optimize the [ssd_mobilenet_v2_coco](https://docs.openvinotoolkit.org/latest/omz_models_public_ssd_mobilenet_v2_coco_ssd_mobilenet_v2_coco.html) object detection model:

        python3 mo_tf.py --input_model <path_to_downloaded_model>/frozen_inference_graph.pb --tensorflow_object_detection_api_pipeline_config <path_to_downloaded_model>/pipeline.config --input_shape [1,300,300,3] --tensorflow_use_custom_operations_config extensions/front/tf/ssd_v2_support.json

    Example:

        python3 mo_tf.py --input_model /opt/models/public/ssd_mobilenet_v2_coco/ssd_mobilenet_v2_coco_2018_03_29/frozen_inference_graph.pb --tensorflow_object_detection_api_pipeline_config /opt/models/public/ssd_mobilenet_v2_coco/ssd_mobilenet_v2_coco_2018_03_29/pipeline.config --input_shape [1,300,300,3] --tensorflow_use_custom_operations_config extensions/front/tf/ssd_v2_support.json

- After successfully optimizing the model, you should see the following results:

        Model Optimizer arguments:
        Common parameters:
            Path to the Input Model:     /opt/models/public/ssd_mobilenet_v2_coco/ssd_mobilenet_v2_coco_2018_03_29/frozen_inference_graph.pb
            Path for generated IR:     /opt/intel/openvino_2021.1.110/deployment_tools/model_optimizer/.
            IR output name:     frozen_inference_graph
            Log level:     ERROR
            Batch:     Not specified, inherited from the model
            Input layers:     Not specified, inherited from the model
            Output layers:     Not specified, inherited from the model
            Input shapes:     [1,300,300,3]
            Mean values:     Not specified
            Scale values:     Not specified
            Scale factor:     Not specified
            Precision of IR:     FP32
            Enable fusing:     True
            Enable grouped convolutions fusing:     True
            Move mean values to preprocess section:     None
            Reverse input channels:     False
        TensorFlow specific parameters:
            Input model in text protobuf format:     False
            Path to model dump for TensorBoard:     None
            List of shared libraries with TensorFlow custom layers implementation:     None
            Update the configuration file with input/output node names:     None
            Use configuration file used to generate the model with Object Detection API:     /opt/models/public/ssd_mobilenet_v2_coco/ssd_mobilenet_v2_coco_2018_03_29/pipeline.config
            Use the config file:     /opt/intel/openvino_2021.1.110/deployment_tools/model_optimizer/extensions/front/tf/ssd_v2_support.json
        Model Optimizer version:     2021.1.0-1237-bece22ac675-releases/2021/1
        The Preprocessor block has been removed. Only nodes performing mean value subtraction and scaling (if applicable) are kept.
        [ SUCCESS ] Generated IR version 10 model.
        [ SUCCESS ] XML file: /opt/intel/openvino_2021.1.110/deployment_tools/model_optimizer/./frozen_inference_graph.xml
        [ SUCCESS ] BIN file: /opt/intel/openvino_2021.1.110/deployment_tools/model_optimizer/./frozen_inference_graph.bin
        [ SUCCESS ] Total execution time: 176.71 seconds. 
        [ SUCCESS ] Memory consumed: 644 MB.


## Copy the optimized artifacts to the model location

The FTL sample project expects the optimized model to be present in the `/opt/models` folder. Copy the optimized frozen inference graph model files to `/opt/models`.

        cp frozen_inference_graph.* /opt/models

For detailed information on the FTL sample project, see the [Getting started](https://github.com/aws-deepracer/aws-deepracer-follow-the-leader-sample-project/blob/main/getting-started.md) section.
