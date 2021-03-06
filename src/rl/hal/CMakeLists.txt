if(MSVC AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 19.00.23918)
	find_package(Boost REQUIRED COMPONENTS thread)
else()
	find_package(Boost REQUIRED)
endif()

find_package(ATIDAQ)
find_package(cifX)
find_package(Comedi)
find_package(libdc1394)

include(TestBigEndian)

test_big_endian(BIG_ENDIAN)

set(
	HDRS
	AnalogInput.h
	AnalogInputReader.h
	AnalogInputWriter.h
	AnalogOutput.h
	AnalogOutputReader.h
	AnalogOutputWriter.h
	AxisController.h
	Camera.h
	CartesianForceSensor.h
	CartesianPositionActuator.h
	CartesianPositionSensor.h
	CartesianVelocitySensor.h
	Coach.h
	ComException.h
	Com.h
	CyclicDevice.h
	DeviceException.h
	Device.h
	DigitalInput.h
	DigitalInputReader.h
	DigitalInputWriter.h
	DigitalOutput.h
	DigitalOutputReader.h
	DigitalOutputWriter.h
	Endian.h
	Exception.h
	Fieldbus.h
	ForceSensor.h
	Gnuplot.h
	Gripper.h
	JointAccelerationActuator.h
	JointCurrentSensor.h
	JointPositionActuator.h
	JointPositionSensor.h
	JointTorqueActuator.h
	JointTorqueSensor.h
	JointVelocityActuator.h
	JointVelocitySensor.h
	LeuzeRs4.h
	Lidar.h
	MitsubishiH7.h
	MitsubishiR3.h
	RangeSensor.h
	SchmersalLss300.h
	SchunkFpsF5.h
	Serial.h
	SickLms200.h
	SickS300.h
	SixAxisForceTorqueSensor.h
	Socket.h
	TimeoutException.h
	TorqueSensor.h
	UniversalRobotsDashboard.h
	UniversalRobotsRealtime.h
	UniversalRobotsRtde.h
	WeissException.h
	WeissKms40.h
	WeissWsg50.h
)

set(
	SRCS
	AnalogInput.cpp
	AnalogInputReader.cpp
	AnalogInputWriter.cpp
	AnalogOutput.cpp
	AnalogOutputReader.cpp
	AnalogOutputWriter.cpp
	AxisController.cpp
	Camera.cpp
	CartesianForceSensor.cpp
	CartesianPositionActuator.cpp
	CartesianPositionSensor.cpp
	CartesianVelocitySensor.cpp
	Coach.cpp
	Com.cpp
	ComException.cpp
	CyclicDevice.cpp
	Device.cpp
	DeviceException.cpp
	DigitalInput.cpp
	DigitalInputReader.cpp
	DigitalInputWriter.cpp
	DigitalOutput.cpp
	DigitalOutputReader.cpp
	DigitalOutputWriter.cpp
	Endian.cpp
	Exception.cpp
	Fieldbus.cpp
	ForceSensor.cpp
	Gnuplot.cpp
	Gripper.cpp
	JointAccelerationActuator.cpp
	JointCurrentSensor.cpp
	JointPositionActuator.cpp
	JointPositionSensor.cpp
	JointTorqueActuator.cpp
	JointTorqueSensor.cpp
	JointVelocityActuator.cpp
	JointVelocitySensor.cpp
	LeuzeRs4.cpp
	Lidar.cpp
	MitsubishiH7.cpp
	MitsubishiR3.cpp
	RangeSensor.cpp
	SchmersalLss300.cpp
	SchunkFpsF5.cpp
	Serial.cpp
	SickLms200.cpp
	SickS300.cpp
	SixAxisForceTorqueSensor.cpp
	Socket.cpp
	TimeoutException.cpp
	TorqueSensor.cpp
	UniversalRobotsDashboard.cpp
	UniversalRobotsRealtime.cpp
	UniversalRobotsRtde.cpp
	WeissException.cpp
	WeissKms40.cpp
	WeissWsg50.cpp
)

if(ATIDAQ_FOUND AND Comedi_FOUND)
	list(APPEND HDRS Ati.h)
	list(APPEND SRCS Ati.cpp)
endif()

if(cifX_FOUND)
	list(APPEND HDRS HilscherCifx.h)
	list(APPEND SRCS HilscherCifx.cpp)
endif()

if(Comedi_FOUND)
	list(APPEND HDRS Comedi.h)
	list(APPEND HDRS Jr3.h)
	list(APPEND SRCS Comedi.cpp)
	list(APPEND SRCS Jr3.cpp)
endif()

if(libdc1394_FOUND)
	list(APPEND HDRS Dc1394Camera.h)
	list(APPEND SRCS Dc1394Camera.cpp)
endif()

add_library(
	hal
	${HDRS}
	${SRCS}
)

target_compile_definitions(
	hal
	PUBLIC
	-DBOOST_ALL_NO_LIB
)

if(MSVC AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 19.00.23918)
	target_compile_definitions(
		hal
		PUBLIC
		-DBOOST_ALL_NO_LIB
		-DBOOST_CHRONO_HEADER_ONLY
		-DBOOST_ERROR_CODE_HEADER_ONLY
		-DBOOST_SYSTEM_NO_DEPRECATED
	)
endif()

if(BIG_ENDIAN)
	target_compile_definitions(hal PRIVATE -DHAVE_BIG_ENDIAN)
else()
	target_compile_definitions(hal PRIVATE -DHAVE_LITTLE_ENDIAN)
endif()

if(NOT CMAKE_VERSION VERSION_LESS 3.8)
	target_compile_features(hal PUBLIC cxx_std_11)
endif()

target_link_libraries(
	hal
	math
	util
)

if(MSVC AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 19.00.23918)
	target_link_libraries(hal ${Boost_LIBRARIES})
endif()

target_include_directories(
	hal
	PUBLIC
	$<BUILD_INTERFACE:${rl_SOURCE_DIR}/src>
	$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}/rl-${VERSION}>
	${Boost_INCLUDE_DIR}
)

if(ATIDAQ_FOUND AND Comedi_FOUND)
	target_include_directories(hal PUBLIC ${ATIDAQ_INCLUDE_DIRS})
	target_link_libraries(hal ${ATIDAQ_LIBRARIES})
endif()

if(cifX_FOUND)
	target_include_directories(hal PUBLIC ${cifX_INCLUDE_DIRS})
	target_link_libraries(hal ${cifX_LIBRARIES})
endif()

if(Comedi_FOUND)
	target_include_directories(hal PUBLIC ${Comedi_INCLUDE_DIRS})
	target_link_libraries(hal ${Comedi_LIBRARIES})
endif()

if(libdc1394_FOUND)
	target_compile_definitions(hal PUBLIC ${libdc1394_DEFINITIONS})
	target_include_directories(hal PUBLIC ${libdc1394_INCLUDE_DIRS})
	target_link_libraries(hal ${libdc1394_LIBRARIES})
endif()

if(QNXNTO)
	target_link_libraries(hal socket)
endif()

if(WIN32)
	target_link_libraries(hal ws2_32)
endif()

set_target_properties(
	hal
	PROPERTIES
	OUTPUT_NAME rlhal
	POSITION_INDEPENDENT_CODE ON
	VERSION ${VERSION}
)

if(WIN32)
	if(BUILD_SHARED_LIBS)
		set_target_properties(
			hal
			PROPERTIES
			DEBUG_POSTFIX d
		)
	else()
		set_target_properties(
			hal
			PROPERTIES
			DEBUG_POSTFIX sd
			MINSIZEREL_POSTFIX s
			RELEASE_POSTFIX s
			RELWITHDEBINFO_POSTFIX s
		)
	endif()
endif()

install(FILES ${HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rl-${VERSION}/rl/hal COMPONENT development)

install(
	TARGETS hal
	EXPORT rl
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT runtime NAMELINK_SKIP
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT runtime
)

if(BUILD_SHARED_LIBS)
	install(
		TARGETS hal
		EXPORT rl
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development NAMELINK_ONLY
	)
endif()

if(MSVC AND BUILD_SHARED_LIBS AND NOT CMAKE_VERSION VERSION_LESS 3.1)
	install(FILES $<TARGET_PDB_FILE:hal> DESTINATION ${CMAKE_INSTALL_BINDIR} CONFIGURATIONS Debug RelWithDebInfo COMPONENT debug)
endif()
