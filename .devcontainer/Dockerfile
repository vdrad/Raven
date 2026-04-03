ARG DOCKER_TAG=v5.5.1
FROM espressif/idf:${DOCKER_TAG}

ENV LC_ALL=C.UTF-8
ENV LANG=C.UTF-8

RUN apt-get update -y && apt-get install -y \
    udev \
    python3-tk \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN /bin/bash -c "source /opt/esp/idf/export.sh && python -m pip install --no-cache-dir customtkinter pandas numpy matplotlib"

RUN echo "source /opt/esp/idf/export.sh > /dev/null 2>&1" >> ~/.bashrc

ENTRYPOINT [ "/opt/esp/entrypoint.sh" ]

CMD ["/bin/bash"]