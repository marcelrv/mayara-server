#!/bin/sh

set -eu

cd "$(dirname "$0")/.."

docker buildx build -f demo/Dockerfile -t marineyachtradar/mayara-server:latest .

echo "Now run the image locally with:

docker run --name mayara-demo -p 6502:6502 marineyachtradar/mayara-server:latest
"

