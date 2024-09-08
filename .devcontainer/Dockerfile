# Use an Ubuntu 20.04 image that supports GCC-9
FROM ubuntu:20.04

# Install dependencies including GCC-9

ENV TZ=America/New_York
RUN apt-get update && \
    apt-get install -y tzdata && \
    ln -fs /usr/share/zoneinfo/$TZ /etc/localtime && \
    dpkg-reconfigure --frontend noninteractive tzdata

RUN apt-get update && apt-get install -y \
    gcc-9 g++-9 \
    build-essential \
    pkg-config \
    unzip \
    wget \
    cmake \
    git \
    libeigen3-dev \
    git-lfs \
    libboost-all-dev \
    libglew-dev \
    libpng-dev \
    libtiff-dev \
    libjpeg-dev \
    libgtk2.0-dev \
    libpcl-dev \
    python3-pip \
    clang-format \
    && pip3 install pre-commit \
    && update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 60 \
                          --slave /usr/bin/g++ g++ /usr/bin/g++-9

# Install OpenCV 4.5.5 # Install OpenCV
 ARG OPENCV_VERSION=4.5.5
 RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.zip --no-check-certificate \
     && unzip opencv.zip
 RUN wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/${OPENCV_VERSION}.zip --no-check-certificate \
     && unzip opencv_contrib.zip
 RUN mkdir opencv_build
 RUN cd opencv_build \
     && cmake -DCMAKE_BUILD_TYPE=Release \
     -DBUILD_EXAMPLES=OFF \
     -DBUILD_DOCS=OFF \
     -DBUILD_TESTS=OFF \
     -DBUILD_PERF_TESTS=OFF \
     -DINSTALL_C_EXAMPLES=OFF \
     -DWITH_QT=OFF \
     -DWITH_GTK=ON \
     -DWITH_EIGEN=OFF \
     -DWITH_FFMPEG=OFF \
     -DWITH_OPENEXR=OFF \
     -DWITH_WEBP=OFF \
     -DWITH_TIFF=ON \
     -DWITH_OPENCL=OFF \
     -DWITH_PNG=ON \
     -DWITH_JPEG=ON \
     -DWITH_OPENJPEG=OFF \
     -DWITH_JASPER=OFF \
     -DWITH_V4L=OFF \
     -DWITH_GSTREAMER=OFF \
     -DWITH_1394=OFF \
     -DWITH_IPP=OFF \
     -DVIDEOIO_ENABLE_PLUGINS=OFF \
     -DWITH_PTHREADS_PF=OFF \
     -DPARALLEL_ENABLE_PLUGINS=OFF \
     -DHIGHGUI_ENABLE_PLUGINS=OFF \
     -DWITH_PROTOBUF=OFF \
     -DBUILD_PROTOBUF=OFF \
     -DOPENCV_DNN_OPENCL=OFF \
     -DBUILD_JAVA=OFF \
     -DBUILD_opencv_python2=OFF \
     -DBUILD_opencv_python3=OFF \
     -DWITH_IMGCODEC_PXM=ON \
     -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib-${OPENCV_VERSION}/modules \
     -DBUILD_opencv_alphamat=OFF \
     -DBUILD_opencv_aruco=OFF \
     -DBUILD_opencv_barcode=OFF \
     -DBUILD_opencv_bgsegm=OFF \
     -DBUILD_opencv_bioinspired=OFF \
     -DWITH_QUIRC=OFF \
     # Required for omnidir
     -DBUILD_opencv_ccalib=ON \
     -DBUILD_opencv_cnn_3dobj=OFF \
     -DBUILD_opencv_cudaarithm=OFF \
     -DBUILD_opencv_cudabgsegm=OFF \
     -DBUILD_opencv_cudacodec=OFF \
     -DBUILD_opencv_cudafeatures2d=OFF \
     -DBUILD_opencv_cudafilters=OFF \
     -DBUILD_opencv_cudaimgproc=OFF \
     -DBUILD_opencv_cudalegacy=OFF \
     -DBUILD_opencv_cudaobjdetect=OFF \
     -DBUILD_opencv_cudaoptflow=OFF \
     -DBUILD_opencv_cudastereo=OFF \
     -DBUILD_opencv_cudawarping=OFF \
     -DBUILD_opencv_cudev=OFF \
     -DBUILD_opencv_cvv=OFF \
     -DBUILD_opencv_datasets=OFF \
     -DBUILD_opencv_dnn_objdetect=OFF \
     -DBUILD_opencv_dnn_superres=OFF \
     -DBUILD_opencv_dnns_easily_fooled=OFF \
     -DBUILD_opencv_dpm=OFF \
     -DBUILD_opencv_face=OFF \
     -DBUILD_opencv_freetype=OFF \
     -DBUILD_opencv_fuzzy=OFF \
     -DBUILD_opencv_hdf=OFF \
     -DBUILD_opencv_hfs=OFF \
     -DBUILD_opencv_img_hash=OFF \
     -DBUILD_opencv_intensity_transform=OFF \
     -DBUILD_opencv_julia=OFF \
     -DBUILD_opencv_line_descriptor=OFF \
     -DBUILD_opencv_matlab=OFF \
     -DBUILD_opencv_mcc=OFF \
     -DBUILD_opencv_optflow=OFF \
     -DBUILD_opencv_ovis=OFF \
     -DBUILD_opencv_phase_unwrapping=OFF \
     -DBUILD_opencv_plot=OFF \
     -DBUILD_opencv_quality=OFF \
     -DBUILD_opencv_rapid=OFF \
     -DBUILD_opencv_reg=OFF \
     -DBUILD_opencv_rgbd=OFF \
     -DBUILD_opencv_saliency=OFF \
     -DBUILD_opencv_sfm=OFF \
     -DBUILD_opencv_shape=OFF \
     -DBUILD_opencv_stereo=OFF \
     -DBUILD_opencv_structured_light=OFF \
     -DBUILD_opencv_superres=OFF \
     -DBUILD_opencv_surface_matching=OFF \
     -DBUILD_opencv_text=OFF \
     -DBUILD_opencv_tracking=OFF \
     -DBUILD_opencv_videostab=OFF \
     -DBUILD_opencv_viz=OFF \
     -DBUILD_opencv_wechat_qrcode=OFF \
     -DBUILD_opencv_xfeatures2d=OFF \
     -DBUILD_opencv_ximgproc=OFF \
     -DBUILD_opencv_xobjdetect=OFF \
     -DBUILD_opencv_xphoto=OFF \
     -DBUILD_SHARED_LIBS=OFF \
     ../opencv-${OPENCV_VERSION} \
     && make -j$(nproc --all) \
     && make install


# Ensure Git is installed
RUN git --version

# Set the working directory
WORKDIR /workspace

RUN git init
# Copy the pre-commit configuration file
COPY .pre-commit-config.yaml /workspace/.pre-commit-config.yaml

# Install pre-commit hooks
RUN pre-commit install --install-hooks

# Copy the project files into the container
# COPY . .

# # Run CMake and build the project
# RUN cmake . && make

# # Run the tests after building
# RUN ctest --output-on-failure

# Default command to run when a container starts
CMD ["/bin/bash"]
