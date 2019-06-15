start-eclipse-project:
	#- mkdir eclipse
	bash -c 'cd eclipse; \
		source /opt/ros/kinetic/setup.bash; \
		cmake -G "Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE -DCMAKE_ECLIPSE_MAKE_ARGUMENTS=-j8 ../src/;'
	# NOTE: Make sure to import an `Existing projects into workspace` on the first time openning eclipse 
