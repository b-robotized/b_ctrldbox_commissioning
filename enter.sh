#!/bin/bash
set -e

# Some validation
if [ ! -f .env ]; then
    echo "ERROR: Configuration file '.env' not found."
    echo "Please copy '.env.example' to '.env' and fill in your details."
    exit 1
fi
export $(grep -v '^#' .env | xargs)

echo "Entering shell in container: ${CONTAINER_NAME}..."

docker exec -it ${CONTAINER_NAME} /bin/bash