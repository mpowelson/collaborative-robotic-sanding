# This is an auto generated Dockerfile for ros:ros-core
# generated from docker_images_ros2/create_ros_core_image.Dockerfile.em
FROM ubuntu:bionic

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && apt-get install -q -y tzdata && rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y \
    dirmngr \
    gnupg2 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
RUN echo "deb http://packages.ros.org/ros2/ubuntu bionic main" > /etc/apt/sources.list.d/ros2-latest.list

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# bootstrap rosdep
RUN rosdep init \
    && rosdep update

# setup colcon mixin and metadata
RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

# install python packages
RUN pip3 install -U \
    argcomplete

# install ros2 packages
ENV ROS_DISTRO eloquent
RUN apt-get update && apt-get install -y \
    ros-eloquent-ros-core=0.8.4-1* \
    && rm -rf /var/lib/apt/lists/*

ENV QT_VERSION_A=5.13
ENV QT_VERSION_B=5.13.2
ENV QT_VERSION_SCRIPT=5132
RUN wget https://download.qt.io/archive/qt/${QT_VERSION_A}/${QT_VERSION_B}/qt-opensource-linux-x64-${QT_VERSION_B}.run
RUN chmod +x qt-opensource-linux-x64-${QT_VERSION_B}.run
COPY qt-noninteractive.qs /qt-noninteractive.qs
RUN ./qt-opensource-linux-x64-${QT_VERSION_B}.run --script qt-noninteractive.qs  -platform minimal

RUN rm /qt-opensource-linux-x64-${QT_VERSION_B}.run
RUN rm /qt-noninteractive.qs

ENV PATH="/qt/${QT_VERSION_B}/gcc_64/lib/cmake:$PATH"
ENV LD_LIBRARY_PATH=/qt/${QT_VERSION_B}/gcc_64/lib:$LD_LIBRARY_PATH

# setup entrypoint
COPY ./ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
