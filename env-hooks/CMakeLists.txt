configure_file(99.tmc_gazebo_task_evaluators.bash.in 99.tmc_gazebo_task_evaluators.bash @ONLY)
catkin_add_env_hooks(99.tmc_gazebo_task_evaluators SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
