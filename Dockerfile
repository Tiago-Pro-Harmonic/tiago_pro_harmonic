FROM osrf/ros:jazzy-desktop-full

ARG DOCKER_USER=user
ARG DOCKER_GROUP=user

RUN apt-get update -y \
 && DEBIAN_FRONTEND=noninteractive apt-get install -qqy curl
RUN mkdir -p /etc/apt/keyrings
RUN curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc | tee /etc/apt/keyrings/robotpkg.asc
RUN echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub noble robotpkg" > /etc/apt/sources.list.d/robotpkg.list
RUN echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/wip/packages/debian/pub noble robotpkg" >> /etc/apt/sources.list.d/robotpkg.list

RUN apt-get update -y && DEBIAN_FRONTEND=noninteractive apt-get install -qqy \
    libgraphviz-dev libqt5svg5-dev pyqt5-dev qtbase5-private-dev \
    robotpkg-py312-pinocchio robotpkg-py312-omniorbpy \
    robotpkg-hpp-statistics+doc \
    robotpkg-hpp-util+doc robotpkg-hpp-pinocchio+doc \
    robotpkg-hpp-constraints+doc robotpkg-hpp-core+doc \
    robotpkg-qt5-qgv \
    libboost-filesystem1.83-dev libboost-python1.83.0 \
    libboost-thread1.83-dev python3-numpy liburdfdom-dev wget python3.12-venv \
    python-is-python3 doxygen \
    tree nano python3-vcstool python3-pip

RUN apt-get update -y && DEBIAN_FRONTEND=noninteractive apt-get install -qqy \
    robotpkg-py312-crocoddyl \
    robotpkg-py312-eigenpy \
    liblapack-dev libblas-dev \
    ros-jazzy-generate-parameter-library \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-launch-param-builder \
    ros-jazzy-realsense2-description \
    ros-jazzy-tf-transformations \
    python3-opencv

RUN apt-get update -y && DEBIAN_FRONTEND=noninteractive apt-get install -qqy \
    libasound2-dev libatk1.0-0 libc6 libcairo-gobject2 libcairo2 libdbus-1-3 libdbus-glib-1-2 \
    libffi8 libfontconfig1 libfreetype6 libglib2.0-0 libgtk-3-0 libnspr4 libnss3 libpango-1.0-0 \
    libstdc++6 libvpx9 apulse \
    libx11-6 libx11-xcb1 libxcb-shm0 libxcb1 libxcomposite1 libxdamage1 libxext6 libxfixes3 \
    libxrandr2 libxtst6 zlib1g fontconfig procps

COPY midori_11.6-1_amd64.deb .
RUN dpkg -i midori_11.6-1_amd64.deb
RUN rm -f midori_11.6-1_amd64.deb

# Add user
RUN addgroup --gid $DOCKER_GROUP user
RUN adduser --uid $DOCKER_USER --gid $DOCKER_GROUP user

USER user
RUN mkdir /home/user/devel

ENV DEVEL_HPP_DIR=/home/user/devel/hpp_ws

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "export ROS_DOMAIN_ID=5" >> ~/.bashrc

