# HERMES
A **Docker Compose** managed development environment that includes the `mavros` and `slam` containers.  
The provided Makefile offers simple commands to quickly build, start, stop, view logs, and access containers.

## Prerequisites
- [Docker](https://docs.docker.com/get-docker/)
- [Docker Compose](https://docs.docker.com/compose/)
- GNU Make

## Initialize Repository
This repository includes `mavros`, `fast-lio2`, and `livox_ros_driver2` as third-party dependencies.  
Before building, make sure to initialize all submodules:
```bash
git submodule update --init --recursive

```

## Usage
From the project root directory, run:
```bash
make <command>
```
Available commands:
```markdown
---------------- < cmd > --------------------
|   build         - Build docker images
|   up            - Compose up containers
|   down          - Compose down containers
|   logs          - Container logs
|   mavros        - Attach `mavros` container    
|   slam          - Attach `slam` container
---------------------------------------------
```

## License
Copyright (c) 2025 Ohin-Kyuu.
All rights reserved.

This repository is private and may only be used by authorized personnel.