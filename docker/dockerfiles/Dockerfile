FROM ros:humble-ros-base AS base
LABEL org.opencontainers.image.authors="ohin.kyuu@gmail.com"
ARG DEBIAN_FRONTEND=noninteractive
ENV TERM=xterm-256color
COPY docker/scripts/build/ /build/
COPY docker/scripts/entrypoint/ros_entrypoint.sh /ros_entrypoint.sh
RUN chown root:root /ros_entrypoint.sh && \
    chmod 755 /ros_entrypoint.sh && \
    sh /build/install_depend.sh

FROM base AS livox-build
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git \
    && cd ./Livox-SDK2 \
    && mkdir build \
    && cd build \
    && cmake .. && make -j \
    && make install

FROM base AS mavros
ARG USER
ARG USER_UID
ARG USER_GID
ENV ROS_WS=/home/${USER}/hermes_ws
RUN sh /build/setup_user.sh ${USER} ${USER_UID} ${USER_GID}
ENTRYPOINT [ "/ros_entrypoint.sh" ]
WORKDIR ${ROS_WS}
COPY packages/mavros_pkg ${ROS_WS}/src
RUN sh /build/rosdep_init.sh ${USER} \
    && cd ${ROS_WS}/src/mavros/scripts \
    && ./install_geographiclib_datasets.sh \
    && sh /build/colcon_build.sh ${USER}
USER $USER
VOLUME ${ROS_WS}/install
CMD [ "/bin/bash" ]

FROM base AS slam
ARG USER
ARG USER_UID
ARG USER_GID
ENV ROS_WS=/home/${USER}/hermes_ws
COPY --from=livox-build /usr/local/lib/liblivox_lidar_sdk_* /usr/local/lib/
COPY --from=livox-build /usr/local/include/livox_lidar_* /usr/local/include/
RUN sh /build/setup_user.sh ${USER} ${USER_UID} ${USER_GID}
ENTRYPOINT [ "/ros_entrypoint.sh" ]
WORKDIR ${ROS_WS}
COPY packages/slam_pkg ${ROS_WS}/src
RUN sh /build/rosdep_init.sh ${USER} \
    && cd ${ROS_WS}/src/livox_ros_driver2 \
    && bash -c "source /opt/ros/humble/setup.bash && ./build.sh humble" \
    && cd ${ROS_WS} \
    && sh /build/colcon_build.sh ${USER}
USER $USER
VOLUME ${ROS_WS}/install
CMD [ "/bin/bash" ]