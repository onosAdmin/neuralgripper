# Use NVIDIA DeepStream as base image
FROM nvcr.io/nvidia/deepstream:6.4-gc-triton-devel as base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

# Install basic dependencies
RUN apt-get update && apt-get install -y \
    curl \
    vim \
    x11-apps \
    gnupg2 \
    lsb-release \
    software-properties-common \
    build-essential \
    cmake \
    git \
    pkg-config \
    unzip \
    yasm \
    checkinstall \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libxvidcore-dev \
    libx264-dev \
    libgtk-3-dev \
    libtbb-dev \
    libatlas-base-dev \
    libfaac-dev \
    libmp3lame-dev \
    libtheora-dev \
    libvorbis-dev \
    libxine2-dev \
    libtbb2 \
    libeigen3-dev \
    python3-dev \
    python3-pip \
    python3-numpy \
    python3-testresources \
    && rm -rf /var/lib/apt/lists/*

# Set up ROS 2 Rolling repositories
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Rolling
RUN apt-get update && apt-get install -y \
    ros-rolling-ros-base \
    ros-rolling-vision-msgs \
    ros-rolling-cv-bridge \
    ros-rolling-tf2-ros \
    ros-rolling-visualization-msgs \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || echo "rosdep already initialized" && rosdep update




# Install CycloneDDS for ros2 external communication
RUN apt-get update && apt-get install -y \
    ros-rolling-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Set CycloneDDS as the default RMW implementation
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///cyclonedds.xml


# Optional pyserial
COPY pyserial-3.5 /pyserial-3.5
RUN cd /pyserial-3.5 && python3 setup.py install




# Install MoveIt 2 dependencies (ROS 2 Rolling)
RUN apt-get update && apt-get install -y \
    ros-rolling-moveit \
    ros-rolling-moveit-ros-visualization \
    ros-rolling-moveit-planners-ompl \
    ros-rolling-moveit-servo \
    ros-rolling-moveit-resources \
    ros-rolling-moveit-visual-tools \
    ros-rolling-ros2-control \
    ros-rolling-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*


#RUN apt-get update && apt-get install --quiet -y --no-install-recommends \
#  gstreamer1.0-gl \
#  gstreamer1.0-opencv \
#  gstreamer1.0-plugins-bad \
#  gstreamer1.0-plugins-good \
#  gstreamer1.0-plugins-ugly \
#  gstreamer1.0-tools \
#  libgstreamer-plugins-base1.0-dev \
#  libgstreamer1.0-0 \
#  libgstreamer1.0-dev



# Create directory for OpenCV build
WORKDIR /opt/build
COPY install_opencv_from_source.sh /opt/build/install_opencv_from_source.sh
RUN chmod +x /opt/build/install_opencv_from_source.sh

RUN bash /opt/build/install_opencv_from_source.sh

# Install PyTorch and YOLO dependencies
RUN pip3 install torch==2.6.0 \
    torchvision==0.21.0 \
    ultralytics==8.3.111 \
    numpy   
    

COPY ros_entrypoint.sh /ros_entrypoint.sh

RUN chmod +x /ros_entrypoint.sh

# Set the entry point
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
