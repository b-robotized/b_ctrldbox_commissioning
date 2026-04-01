#!/bin/bash
set -e

# Some validation
if ls *.env 1> /dev/null 2>&1; then
  ENV_FILE=$(find *.env)
  source <(grep -v '^#' $ENV_FILE)
else
  echo "ERROR: Configuration file '*.env' not found."
  echo "Please copy 'commissioning.env.example' to '*.env' and fill in your details."
  exit 1
fi

if [ -z "$HOST_NETWORK_INTERFACE" ]; then
    echo "ERROR: HOST_NETWORK_INTERFACE is not set in your .env file."
    echo "Please define the HOST_NETWORK_INTERFACE variable."
    echo "This is the network interface on your computer that is connected to the 192.168.28.x network."
    echo "Run 'ip addr' in your terminal to find this. Look for an interface with an IP in that range."
    exit 1
fi

export IMAGE_REGISTRY_URL=code.b-robotized.com:5050/b_public/b_products/b_controlled_box/b-controlled-box-commissioning-containers
export IMAGE_NAME=${ROBOT_TYPE}-commission
export IMAGE_URL_FULL=${IMAGE_REGISTRY_URL}/${IMAGE_NAME}:${VERSION_TAG}
export IMAGE_TAG_FINAL=b-robotized/${IMAGE_NAME}:${VERSION_TAG}


if [ -z "$CONTAINER_NAME" ]; then
    export CONTAINER_NAME=b-robotized-${IMAGE_NAME}
fi