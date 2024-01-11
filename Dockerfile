FROM ros:humble
ARG USERNAME=element
ARG USER_UID=1000
ARG USER_GID=$USER_UID


RUN apt-get update && apt-get install -y \
    ros-humble-ros-gz \
    ros-humble-navigation2 \
    ros-humble-tf-transformations \
    ros-humble-robot-localization \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir transforms3d

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME

RUN mkdir -p /home/${USERNAME}/ros_ws/src
RUN chown -R element:element /home/${USERNAME}/ros_ws
# RUN git clone https://github.com/ecthelion99/element_robotics_task.git
COPY . /home/${USERNAME}/ros_ws/src/
ENV SHELL /bin/bash

# ********************************************************
# ********************************************************

USER $USERNAME
RUN /bin/bash -c '. /opt/ros/humble/setup.bash; cd /home/${USERNAME}/ros_ws; colcon build'
RUN echo "source /home/${USERNAME}/ros_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc
CMD ["/bin/bash"]