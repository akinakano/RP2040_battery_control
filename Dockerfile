FROM ubuntu:24.04

ENV DEBIAN_FRONTEND noninteractive
ENV HTTP_PROXY "http://proxy.kanto.sony.co.jp:10080"
ENV http_proxy "http://proxy.kanto.sony.co.jp:10080"
ENV HTTPS_PROXY "http://proxy.kanto.sony.co.jp:10080"
ARG ARCH
RUN apt update && apt upgrade -y && \
    apt install -y build-essential wget
RUN mkdir -p /usr/local/arm-non-eabi
RUN wget -O - https://github.com/xpack-dev-tools/arm-none-eabi-gcc-xpack/releases/download/v13.2.1-1.1/xpack-arm-none-eabi-gcc-13.2.1-1.1-linux-${ARCH}.tar.gz | tar zxvf - -C /usr/local/arm-non-eabi --strip-components 1
RUN mkdir -p /primo
WORKDIR "/primo"

ENV PATH "/usr/local/arm-non-eabi/bin:${PATH}"

