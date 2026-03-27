#!/bin/bash
set -e
source ./common.sh

# Pull image on first start
if [[ "$(docker images -q ${IMAGE_URL_FULL} 2> /dev/null)" == "" ]]; then
  echo "Image '${IMAGE_URL_FULL}' not found locally. Pulling from registry..."
  docker pull ${IMAGE_URL_FULL}
else
  echo "Image '${IMAGE_URL_FULL}' found locally. Skipping pull."
fi

docker tag ${IMAGE_URL_FULL} ${IMAGE_TAG_FINAL}

# start the container
echo "Starting the ROS container environment..."
echo ".env file: $ENV_FILE"
docker compose --env-file $ENV_FILE up -d 

echo "✅ Environment is up and running!"
echo "To access the container shell, run enter.sh script!"