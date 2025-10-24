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

if [ -z "$IMAGE_REGISTRY_URL" ]; then
    echo "ERROR: IMAGE_REGISTRY_URL is missing in your .env file."
    exit 1
fi

# Pull image on first start
if [[ "$(docker images -q ${IMAGE_TAG_FULL} 2> /dev/null)" == "" ]]; then
  echo "Image '${IMAGE_TAG_FULL}' not found locally. Pulling from registry..."

  # Login to the private registry using the credentials from the .env file
  docker login ${IMAGE_REGISTRY_URL}

  docker pull ${IMAGE_TAG_FULL}
else
  echo "Image '${IMAGE_TAG_FULL}' found locally. Skipping pull."
fi

# start the container
echo "Starting the ROS container environment..."
echo ".env file: $ENV_FILE"
docker compose --env-file $ENV_FILE up -d 

echo "✅ Environment is up and running!"
echo "To access the container shell, run enter.sh script!"