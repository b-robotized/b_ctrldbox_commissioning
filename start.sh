#!/bin/bash
set -e

# Some validation
if [ ! -f .env ]; then
    echo "ERROR: Configuration file '.env' not found."
    echo "Please copy '.env.example' to '.env' and fill in your details."
    exit 1
fi
export $(grep -v '^#' .env | xargs)

if [ -z "$IMAGE_TAG_FULL" ] || [ -z "$IMAGE_REGISTRY_URL" ] || [ -z "$IMAGE_REGISTRY_ACCESS_USER" ] || [ -z "$IMAGE_REGISTRY_ACCESS_TOKEN" ]; then
    echo "ERROR: One or more required variables are missing in your .env file."
    echo "Please ensure IMAGE_REGISTRY_URL, IMAGE_REGISTRY_ACCESS_USER, IMAGE_REGISTRY_ACCESS_TOKEN, and IMAGE_TAG_FULL are all set."
    exit 1
fi

# Pull image on first start
if [[ "$(docker images -q ${IMAGE_TAG_FULL} 2> /dev/null)" == "" ]]; then
  echo "Image '${IMAGE_TAG_FULL}' not found locally. Pulling from registry..."

  # Login to the private registry using the credentials from the .env file
  # The password is piped from stdin for better security than command-line args
  echo "${IMAGE_REGISTRY_ACCESS_TOKEN}" | docker login ${IMAGE_REGISTRY_URL} --username ${IMAGE_REGISTRY_ACCESS_USER} --password-stdin

  docker pull ${IMAGE_REGISTRY_URL}/${IMAGE_TAG_FULL}
else
  echo "Image '${IMAGE_TAG_FULL}' found locally. Skipping pull."
fi

# start the container
echo "Starting the ROS container environment..."
docker compose up -d

echo "✅ Environment is up and running!"
echo "To access the container shell, run enter.sh script!"