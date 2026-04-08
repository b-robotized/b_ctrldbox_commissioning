#!/bin/bash
set -e

# ---------------------------------------------------------------------------
# common.sh -- Shared configuration for all container lifecycle scripts.
#
# Reads the .env file, validates required variables, and exports the image
# and container names used by start.sh, enter.sh, stop.sh, and docker-compose.
# ---------------------------------------------------------------------------

if ls *.env 1> /dev/null 2>&1; then
  ENV_FILE=$(find *.env)
  source <(grep -v '^#' $ENV_FILE)
else
  echo "ERROR: Configuration file '*.env' not found."
  echo "Please copy 'comissioning.env.example' to '<your-name>.env' and fill in your details."
  exit 1
fi

if [ -z "$HOST_NETWORK_INTERFACE" ]; then
    echo "ERROR: HOST_NETWORK_INTERFACE is not set in your .env file."
    echo "Please define the HOST_NETWORK_INTERFACE variable."
    echo "This is the network interface on your computer that is connected to the 192.168.28.x network."
    echo "Run 'ip addr' in your terminal to find this. Look for an interface with an IP in that range."
    exit 1
fi

# --- Determine the container registry URL ---
# By default, images are pulled from the public registry. If you have been
# provided a private image and an access token by b-robotized, set
# USE_PRIVATE_REGISTRY=true in your .env file and authenticate with
#
# echo "<b-robotized-access-token>" | docker login code.b-robotized.com:5050 -u ctrlx --password-stdin
#
# before running start.sh.
IMAGE_REGISTRY_URL_PUBLIC="code.b-robotized.com:5050/b_public/b_products/b_controlled_box/b-controlled-box-commissioning-containers"
IMAGE_REGISTRY_URL_PRIVATE="code.b-robotized.com:5050/b-controlled-box/releases"

if [ "$USE_PRIVATE_REGISTRY" = "true" ]; then
  export IMAGE_REGISTRY_URL="${IMAGE_REGISTRY_URL_PRIVATE}"
else
  export IMAGE_REGISTRY_URL="${IMAGE_REGISTRY_URL_PUBLIC}"
fi

export IMAGE_NAME=${ROBOT_TYPE}-commission
export IMAGE_URL_FULL=${IMAGE_REGISTRY_URL}/${IMAGE_NAME}:${VERSION_TAG}
export IMAGE_TAG_FINAL=b-robotized/${IMAGE_NAME}:${VERSION_TAG}

if [ -z "$CONTAINER_NAME" ]; then
    export CONTAINER_NAME=b-robotized-${IMAGE_NAME}
fi