#!/bin/bash
set -e
source ./common.sh

echo "Stopping and removing the container environment..."
docker compose --env-file "$ENV_FILE" down
echo "Environment stopped."