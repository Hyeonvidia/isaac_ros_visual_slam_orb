SHELL := /bin/bash
SCRIPTS := scripts

.DEFAULT_GOAL := stereo
.PHONY: help mono mono-imu stereo stereo-imu rgbd rgbd-imu shell build down

help: ## Show this help
	@echo "=============================================="
	@echo " ORB-SLAM3 RealSense D456 - Makefile"
	@echo "=============================================="
	@echo ""
	@echo "Usage: make <target>"
	@echo ""
	@echo "Targets:"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | \
		awk 'BEGIN {FS = ":.*?## "}; {printf "  make %-14s %s\n", $$1, $$2}'
	@echo ""
	@echo "Sensor modes:"
	@echo "  mono              Left IR only"
	@echo "  mono-imu          Left IR + IMU (experimental)"
	@echo "  stereo   (default) Left + Right IR"
	@echo "  stereo-imu        Left + Right IR + IMU (experimental)"
	@echo "  rgbd              Color + Depth (aligned)"
	@echo "  rgbd-imu          Color + Depth + IMU (experimental)"
	@echo ""
	@echo "Examples:"
	@echo "  make               # same as 'make stereo'"
	@echo "  make rgbd          # RGB-D mode"
	@echo "  make shell         # debug inside container"
	@echo "  make build         # rebuild deploy docker image"
	@echo "  make down          # stop & remove running container"

mono: ## Monocular (left IR only)
	$(SCRIPTS)/run_deploy_orb.sh --mode mono

mono-imu: ## Mono + IMU (experimental)
	$(SCRIPTS)/run_deploy_orb.sh --mode mono-imu

stereo: ## Stereo IR (default)
	$(SCRIPTS)/run_deploy_orb.sh --mode stereo

stereo-imu: ## Stereo + IMU (experimental)
	$(SCRIPTS)/run_deploy_orb.sh --mode stereo-imu

rgbd: ## RGB-D (color + depth)
	$(SCRIPTS)/run_deploy_orb.sh --mode rgbd

rgbd-imu: ## RGB-D + IMU (experimental)
	$(SCRIPTS)/run_deploy_orb.sh --mode rgbd-imu

shell: ## Interactive debug shell
	$(SCRIPTS)/run_deploy_orb.sh -s

build: ## Rebuild deploy image via build_deploy.sh
	bash $(SCRIPTS)/build_deploy.sh

down: ## Stop and remove running container
	@CONTAINER="isaac_ros_visual_slam_orb_deploy-container"; \
	if [ "$$(docker ps -q --filter name=$$CONTAINER)" ]; then \
		echo "Stopping $$CONTAINER..."; \
		docker stop $$CONTAINER; \
	fi; \
	if [ "$$(docker ps -aq --filter name=$$CONTAINER)" ]; then \
		echo "Removing $$CONTAINER..."; \
		docker rm $$CONTAINER; \
	else \
		echo "No container named $$CONTAINER found."; \
	fi
