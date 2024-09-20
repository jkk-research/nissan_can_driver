#!/bin/bash

cd ../../
docker build --platform linux/arm64 -t nissan_can_arm64 -f nissan_can_driver/docker/Dockerfile .
