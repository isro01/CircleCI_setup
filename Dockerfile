
FROM ubuntu:18.04

RUN \
    apt-get update && \
    apt-get -y upgrade && \
    apt-get -y install git && \
    apt-get install -y python python-pip python-dev && \
    python -m pip install cpplint

