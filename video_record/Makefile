IMAGE_TAG ?= latest
GIT_COMMIT = $(shell eval echo `git rev-parse HEAD`)
GIT_BRANCH = $(shell eval echo `git rev-parse --abbrev-ref HEAD`)
GIT_REPO_URL = $(shell eval echo `git config --get remote.origin.url`)

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
		cmake ../src/; \
		make'

build-docker:
	sudo docker build -f Dockerfile -t techye/video_record:$(IMAGE_TAG) .

push-docker:
	sudo docker push techye/video_record:$(IMAGE_TAG) 

clean-docker:
	sudo docker rmi techye/video_record:$(IMAGE_TAG)

run-interactive:
	sudo docker run -it techye/video_record bash

run-compose:
	sudo docker-compose -f docker-compose.yaml up --force-recreate -d
