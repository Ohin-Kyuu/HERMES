UID := $(shell id -u)
GID := $(shell id -g)
build:
	UID=$(UID) GID=$(GID) docker compose -f docker/composes/compose.dev.yml build

up:
	UID=$(UID) GID=$(GID) docker compose -f docker/composes/compose.dev.yml up -d

down:
	UID=$(UID) GID=$(GID) docker compose -f docker/composes/compose.dev.yml down

logs:
	UID=$(UID) GID=$(GID) docker compose -f docker/composes/compose.dev.yml logs -f

mavros:
	UID=$(UID) GID=$(GID) docker exec -it mavros bash

slam:
	UID=$(UID) GID=$(GID) docker exec -it slam bash

buildx:
	@echo "==> Building hermes/mavros:latest (multi-arch)"
	docker buildx build \
		--platform linux/amd64,linux/arm64 \
		-f docker/dockerfiles/Dockerfile \
		--target mavros \
		--build-arg USER=mavros \
		--build-arg USER_UID=$(UID) \
		--build-arg USER_GID=$(GID) \
		-t ohin112/mavros-ros2:latest \
		--push .

	@echo "==> Building hermes/slam:latest (multi-arch)"
	docker buildx build \
		--platform linux/amd64,linux/arm64 \
		-f docker/dockerfiles/Dockerfile \
		--target slam \
		--build-arg USER=slam \
		--build-arg USER_UID=$(UID) \
		--build-arg USER_GID=$(GID) \
		-t ohin112/slam-ros2:latest \
		--push .