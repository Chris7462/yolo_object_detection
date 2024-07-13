# YOLO Object Detection

## Preparation

### Download the YOLO Model
See the [YOLO Inference](https://github.com/Chris7462/yolo_inference) for exporting YOLO models.

### Enable CUDA Support for OpenCV
You can safely skip this step if you don't need CUDA support.

To enable GPU support for OpenCV's DNN module in Ubuntu, you'll need to build OpenCV from source with CUDA support. Here's a step-by-step guide:

#### Clone the Repository for OpenCV and OpenCV's Extra Modules
```bash
git clone https://github.com/opencv/opencv.git
git clone https://github.com:opencv/opencv_contrib.git
```

#### Build from the source
```bash
cd opencv
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=RELEASE \
      -DCMAKE_INSTALL_PREFIX=/usr/local \
      -DCUDA_ARCH_BIN=7.5 \
      -DENABLE_CUDA_FIRST_CLASS_LANGUAGE=ON \
      -DOPENCV_DNN_CUDA=ON \
      -DWITH_CUBLAS=1 \
      -DWITH_CUDA=ON \
      -DWITH_CUDNN=ON \
      -DCUDA_FAST_MATH=1 \
      -DENABLE_FAST_MATH=1 \
      -DHAVE_opencv_python3=ON \
      -DWITH_TBB=ON \
      -DBUILD_opencv_sfm=ON \
      -DBUILD_opencv_cudacodec=ON \
      -DBUILD_EXAMPLES=ON \
      -DINSTALL_PYTHON_EXAMPLES=ON \
      -DINSTALL_C_EXAMPLES=ON \
      -DOPENCV_ENABLE_NONFREE=ON \
      -DOPENCV_EXTRA_MODULES_PATH=~/thirdparty/opencv_contrib/modules ..
```
The CMake arg CUDA\_ARCH\_BIN is depending on your nVidia graphic card. Check [wiki](https://en.wikipedia.org/wiki/CUDA) for the reference.

You can trun off flags not related to CUDA, but generally, the flags related to CUDA must be enable, and `CUDA_ARCH_BIN` should be specified. Then, run the following commands:
```bash
make -j4 # or -j${nproc} if you have more cpus
sudo make install
sudo ldconfig
```


### Download the KITTI ROS2 bag
I have created a ROS2 bag from the [KITTI dataset](https://www.cvlibs.net/datasets/kitti/setup.php). You can download it from here: [2011\_09\_29\_drive\_0071_sync_bag.tar.zst](https://drive.google.com/file/d/1PZ7xIgH5Ja_kpzbRrAyZVJ_6ojxR6btw/view?usp=drive_link)

To uncompress the zst file, use the following command:
```bash
tar --zstd -xf 2011_09_29_drive_0071_sync_bag.tar.zst
```

## How to run
### Clone this repository
```bash
git clone https://github.com/Chris7462/yolov_object_detection.git
```

### Build the repository
```bash
colcon build --symlink-install
source ./install/setup.bash
```

### Launch it
```bash
ros2 launch yolov_object_detection yolov_object_detection_rviz_launch.py
```

<!-- g++ -O3 cpp/yolo.cpp -o yolo_example -I/usr/local/include/opencv4 -L/usr/local/lib -lopencv_stitching -lopencv_alphamat -lopencv_aruco -lopencv_barcode -lopencv_bgsegm -lopencv_bioinspired -lopencv_ccalib -lopencv_dnn_objdetect -lopencv_dnn_superres -lopencv_dpm -lopencv_face -lopencv_freetype -lopencv_fuzzy -lopencv_hdf -lopencv_hfs -lopencv_img_hash -lopencv_intensity_transform -lopencv_line_descriptor -lopencv_mcc -lopencv_quality -lopencv_rapid -lopencv_reg -lopencv_rgbd -lopencv_saliency -lopencv_shape -lopencv_stereo -lopencv_structured_light -lopencv_phase_unwrapping -lopencv_superres -lopencv_optflow -lopencv_surface_matching -lopencv_tracking -lopencv_highgui -lopencv_datasets -lopencv_text -lopencv_plot -lopencv_ml -lopencv_videostab -lopencv_videoio -lopencv_viz -lopencv_wechat_qrcode -lopencv_ximgproc -lopencv_video -lopencv_xobjdetect -lopencv_objdetect -lopencv_calib3d -lopencv_imgcodecs -lopencv_features2d -lopencv_dnn -lopencv_flann -lopencv_xphoto -lopencv_photo -lopencv_imgproc -lopencv_core -->
