cmake_minimum_required(VERSION 3.1)
px4_add_module(
	MODULE modules__target_follower
	MAIN target_follower
	SRCS
		target_follower.cpp
	DEPENDS
)
