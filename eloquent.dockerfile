FROM nvidia/cuda:10.0-cudnn7-devel-ubuntu18.04
MAINTAINER Levi Armstrong

ARG DEBIAN_FRONTEND=noninteractive

# setup keys
RUN apt-key adv --keyserver ha.pool.sks-keyservers.net --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list

# install bootstrap tools
RUN apt-get update && apt-get upgrade --no-install-recommends -y && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# bootstrap rosdep
RUN rosdep init \
    && rosdep update

RUN echo "source /opt/ros/eloquent/setup.bash" >> ~/.bashrc

# install ros packages
ENV ROS_DISTRO eloquent
RUN apt-get update && apt-get install -y \
    ros-eloquet-ros-core \
    ros-eloquent-ros-base \
    ros-eloquent-robot

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-eloquent-industrial-core \
    python-wstool \
    python-catkin-tools \
    liblapack-dev \
    libblas-dev

RUN git clone --branch v8.2.0 https://github.com/Kitware/VTK.git
RUN mkdir /vtk_build && \
    cd /vtk_build && \
    cmake -DCMAKE_BUILD_TYPE=Release ../VTK && \
    make -j8 && \
    make install

RUN git clone --branch pcl-1.9.1 https://github.com/PointCloudLibrary/pcl.git
RUN mkdir /pcl_build && \
    cd /pcl_build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_surface_on_nurbs:BOOL=ON ../pcl && \
    make -j8 && \
    make install

RUN rm -rf /VTK /pcl /pcl_build /vtk_build

# install bootstrap tools
RUN apt-get install --no-install-recommends -y unzip wget

ENV QT_VERSION_A=5.13
ENV QT_VERSION_B=5.13.2
ENV QT_VERSION_SCRIPT=5132
RUN wget https://download.qt.io/archive/qt/${QT_VERSION_A}/${QT_VERSION_B}/qt-opensource-linux-x64-${QT_VERSION_B}.run
RUN chmod +x qt-opensource-linux-x64-${QT_VERSION_B}.run
COPY qt-noninteractive.qs /qt-noninteractive.qs
RUN ./qt-opensource-linux-x64-${QT_VERSION_B}.run --script qt-noninteractive.qs  -platform minimal

RUN rm /qt-opensource-linux-x64-${QT_VERSION_B}.run
RUN rm /qt-noninteractive.qs

RUN apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# This is required for yak because it is looking for libcuda.so.1 for some reason instead of libcuda.so
RUN ln -s /usr/local/cuda/lib64/stubs/libcuda.so /usr/local/cuda/lib64/stubs/libcuda.so.1

ENV PATH="/qt/${QT_VERSION_B}/gcc_64/lib/cmake:$PATH"
ENV LD_LIBRARY_PATH=/qt/${QT_VERSION_B}/gcc_64/lib:/usr/local/cuda/lib64/stubs:$LD_LIBRARY_PATH

CMD ["bash"]

