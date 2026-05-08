FROM schmittle/mushr_sim:stable

USER root
#replace with your uid/gid
RUN export uid=1001 gid=1001 && \
    echo "developer:x:${uid}:${gid}:Developer,,,:/home/developer:/bin/bash" >> /etc/passwd && \
    mkdir -p /etc/sudoers.d/ && \
    echo "developer:x:${uid}:" >> /etc/group && \
    echo "developer ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/developer && \
    chmod 0440 /etc/sudoers.d/developer && \
    chown ${uid}:${gid} -R /home/developer

USER developer
ENV HOME /home/developer
CMD /bin/bash -c ". /opt/ros/melodic/setup.bash"
