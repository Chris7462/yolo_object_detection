To enable GPU support for OpenCV's DNN module in Ubuntu, you'll need to build OpenCV from source with CUDA support. Here's a step-by-step guide:

OpenCV & OpenCV Contrib
```bash
git clone git@github.com:opencv/opencv.git
git clone git@github.com:opencv/opencv_contrib.git
```

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


<!-- g++ -O3 cpp/yolo.cpp -o yolo_example -I/usr/local/include/opencv4 -L/usr/local/lib -lopencv_stitching -lopencv_alphamat -lopencv_aruco -lopencv_barcode -lopencv_bgsegm -lopencv_bioinspired -lopencv_ccalib -lopencv_dnn_objdetect -lopencv_dnn_superres -lopencv_dpm -lopencv_face -lopencv_freetype -lopencv_fuzzy -lopencv_hdf -lopencv_hfs -lopencv_img_hash -lopencv_intensity_transform -lopencv_line_descriptor -lopencv_mcc -lopencv_quality -lopencv_rapid -lopencv_reg -lopencv_rgbd -lopencv_saliency -lopencv_shape -lopencv_stereo -lopencv_structured_light -lopencv_phase_unwrapping -lopencv_superres -lopencv_optflow -lopencv_surface_matching -lopencv_tracking -lopencv_highgui -lopencv_datasets -lopencv_text -lopencv_plot -lopencv_ml -lopencv_videostab -lopencv_videoio -lopencv_viz -lopencv_wechat_qrcode -lopencv_ximgproc -lopencv_video -lopencv_xobjdetect -lopencv_objdetect -lopencv_calib3d -lopencv_imgcodecs -lopencv_features2d -lopencv_dnn -lopencv_flann -lopencv_xphoto -lopencv_photo -lopencv_imgproc -lopencv_core -->

