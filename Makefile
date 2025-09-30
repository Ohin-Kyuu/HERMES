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
