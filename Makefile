.PHONY: help up down build rebuild logs single test record

# Show available targets (default)
help:
	@echo "Usage: make <target>"
	@echo ""
	@echo "  up        Start the full 4-robot stack (docker compose up)"
	@echo "  down      Stop and remove containers"
	@echo "  build     Build the sim image"
	@echo "  rebuild   Build then start (build + up)"
	@echo "  logs      Follow logs from all running services"
	@echo "  test      Fast headless regression: validates launch files and imports"
	@echo "  single    Single-robot live test (Gazebo + 1 robot, profile: single)"
	@echo "  record    Run headless at max speed and record a bag"
	@echo "            Pass FOXGLOVE=1 to also start the Foxglove bridge"
	@echo "            Pass EXPLORE=N to stop at N% exploration (e.g. make record EXPLORE=60)"
	@echo "            Pass TIMEOUT=N to stop after N seconds (e.g. make record TIMEOUT=300)"

up:
	docker compose up

down:
	docker compose down

build:
	docker compose build

rebuild: build up

logs:
	docker compose logs -f

test:
	docker build --target test .

single:
	docker compose --profile single up

FOXGLOVE ?=
_FOXGLOVE_PROFILE = $(if $(FOXGLOVE),--profile foxglove,)

EXPLORE ?=
export EXPLORE_CUTOFF = $(or $(EXPLORE),0)
TIMEOUT ?=
export TIMEOUT_CUTOFF = $(or $(TIMEOUT),0)

record:
	docker compose -f compose.yaml -f compose.record.yaml $(_FOXGLOVE_PROFILE) up --build --abort-on-container-exit --exit-code-from recorder
