FROM eduartrobotik/eduart-nodered:0.3.0


RUN mkdir -p /astro_ws/src


WORKDIR /astro_ws/src
copy astro_action_interfaces astro_action_interfaces
RUN cd /astro_ws && \
source /opt/ros/$ROS_DISTRO/setup.bash  \
&& colcon build 

# Enable overlay execution
WORKDIR /
COPY docker/entrypoint.bash .
RUN chmod +x /entrypoint.bash

# CMD [ "node", "/usr/bin/node-red" ]
CMD /usr/bin/node-red
ENTRYPOINT [ "/entrypoint.bash" ]
