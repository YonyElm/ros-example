IMAGE_TAG ?= latest
DOCKER_BUILD_FLAGS ?= # --build-arg CPP_MAKE_FLAGS can be used

GIT_COMMIT ?= $(shell eval echo `git rev-parse HEAD`)
GIT_BRANCH ?= $(shell eval echo `git rev-parse --abbrev-ref HEAD`)
GIT_REPO_URL ?= $(shell eval echo `git config --get remote.origin.url`)
GIT_TAG ?= $(shell eval echo `git tag --points-at HEAD | head -n 1`)

start-eclipse-project:
	- mkdir eclipse
	bash -c 'cd eclipse; \
		source /opt/ros/kinetic/setup.bash; \
		cmake -G "Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE -DCMAKE_ECLIPSE_MAKE_ARGUMENTS=-j8 ../src/;'
	# NOTE: Make sure to import an `Existing projects into workspace` on the first time openning eclipse 

run-compilation:
	- mkdir build
	bash -c 'cd build; \
		source /opt/ros/kinetic/setup.bash; \
		cmake ../src/ -DCMAKE_BUILD_TYPE=Debug; \
		make'

build-docker:
	sudo docker build \
		--build-arg GIT_COMMIT=${GIT_COMMIT} \
		--build-arg GIT_BRANCH=${GIT_BRANCH} \
		--build-arg GIT_REPO_URL=${GIT_REPO_URL} \
		--build-arg GIT_TAG=${GIT_TAG} \
		${DOCKER_BUILD_FLAGS} -f Dockerfile -t techye/view_rosbag:$(IMAGE_TAG) .

push-docker:
	sudo docker push techye/view_rosbag:$(IMAGE_TAG)

clean-docker:
	sudo docker rmi techye/view_rosbag:$(IMAGE_TAG)

run-interactive:
	sudo docker run -it techye/view_rosbag bash

run-compose:
	sudo docker-compose -f docker-compose.yaml up --force-recreate -d