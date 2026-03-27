#!/bin/bash
set -e
source ./common.sh

echo "Entering shell in container: ${CONTAINER_NAME}..."
docker exec -it ${CONTAINER_NAME} /bin/bash