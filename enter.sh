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

echo "Entering shell in container: ${CONTAINER_NAME}..."

docker exec -it ${CONTAINER_NAME} /bin/bash