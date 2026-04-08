#!/bin/bash
set -e
source ./common.sh

# ---------------------------------------------------------------------------
# Image resolution: prefer a locally available image, pull only if missing.
# ---------------------------------------------------------------------------
if [[ "$(docker images -q ${IMAGE_URL_FULL} 2> /dev/null)" == "" ]]; then
  echo "Image '${IMAGE_URL_FULL}' not found locally. Pulling from registry..."

  if [ "$USE_PRIVATE_REGISTRY" = "true" ]; then
    echo "NOTE: You are using a private registry. Make sure you have authenticated first:"
    echo "  echo \"<deploy-token>\" | docker login code.b-robotized.com:5050 -u ctrlx --password-stdin"
    echo ""
  fi

  docker pull ${IMAGE_URL_FULL}
else
  echo "Image '${IMAGE_URL_FULL}' found locally. Using the local image (skipping pull)."
fi

# Re-tag the image to a uniform local name used by docker-compose.
docker tag ${IMAGE_URL_FULL} ${IMAGE_TAG_FINAL}

echo "Starting the ROS container environment..."
echo "  From URL:      $IMAGE_URL_FULL"
echo "  .env file:     $ENV_FILE"
echo "  Re-tagged to:  $IMAGE_TAG_FINAL"
echo "  Container:     $CONTAINER_NAME"
docker compose --env-file $ENV_FILE up -d

echo ""
echo "Environment is up and running!"
echo "To access the container shell, run: ./enter.sh"