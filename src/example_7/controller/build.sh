#! /bin/bash

dir="/home/zerodaytea/src/ros2_control_demos/example_7/controller"
out_dir="/home/zerodaytea/research/ros2-control-vuln-seeding/"

# $dir/c0/build_controller.sh $dir

gcc -c -o $dir/c0/controller.o $dir/c0/controller.c
g++ -c -o $dir/c0/controller_main.o $dir/c0/controller_main.cpp
g++ -o $dir/tmp0 $dir/c0/controller.o $dir/c0/controller_main.o

gcc -c -o $dir/c1/controller.o $dir/c1/controller.c
g++ -c -o $dir/c1/controller_main.o $dir/c1/controller_main.cpp
g++ -o $dir/tmp1 $dir/c0/controller.o $dir/c1/controller_main.o

gcc -c -o $dir/c2/controller.o $dir/c2/controller.c
g++ -c -o $dir/c2/controller_main.o $dir/c2/controller_main.cpp
g++ -o $dir/tmp2 $dir/c2/controller.o $dir/c2/controller_main.o

*redacted*
