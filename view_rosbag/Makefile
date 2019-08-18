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
	sudo docker build -f Dockerfile -t techye/view_rosbag:latest .

run-interactive:
	sudo docker run -it techye/view_rosbag bash

run-compose:
	sudo docker-compose -f docker-compose.yaml up --force-recreate -d