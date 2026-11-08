# Sources and licenses

- Autoware CenterPoint base v4.1 ONNX and configuration: https://huggingface.co/AutowareFoundation/lidar_centerpoint/tree/v4.1 ; Apache-2.0 model card, training-data terms as linked there.
- Native feature/decode conventions: TIER IV / NVIDIA autoware_lidar_centerpoint, Apache-2.0: https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_lidar_centerpoint . Standalone native implementation is local; it does not vendor the Autoware runtime.
- HARP-NeXt network and checkpoint: https://github.com/SamirAbouHaidar/HARP-NeXt and https://huggingface.co/samir-abh/HARP-NeXt ; Apache-2.0. Vendor LICENSE is retained. Original source is preserved as `.upstream`; executable copies only adjust absolute imports and route scatter calls through the adapter. The checkpoint net tensors are extracted unchanged, without retraining.
- Patchwork++: https://github.com/url-kaist/patchwork-plusplus ; installed as pypatchworkpp, following its package license.
- NumPy, SciPy, PyTorch, ROS2 and TensorRT retain their upstream licenses. Source implementation in this new package is Apache-2.0; existing project packages are not relicensed or modified.
