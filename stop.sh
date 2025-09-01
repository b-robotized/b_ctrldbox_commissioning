#!/bin/bash
set -e

echo "Stopping and removing the container environment..."
docker compose down
echo "Environment stopped."