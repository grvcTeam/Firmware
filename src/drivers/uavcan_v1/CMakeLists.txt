############################################################################
#
#   Copyright (c) 2020 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

set(LIBCANARD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libcanard)
set(DSDL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/public_regulated_data_types)

px4_add_git_submodule(TARGET git_libcanard PATH ${LIBCANARD_DIR})
px4_add_git_submodule(TARGET git_public_regulated_data_types PATH ${DSDL_DIR})

execute_process(COMMAND nnvg --templates ${CMAKE_CURRENT_SOURCE_DIR}/templates --outdir ${CMAKE_CURRENT_BINARY_DIR}/dsdlc_generated --target-language cpp -I ${DSDL_DIR}/uavcan ${DSDL_DIR}/regulated)
execute_process(COMMAND nnvg --templates ${CMAKE_CURRENT_SOURCE_DIR}/templates --outdir ${CMAKE_CURRENT_BINARY_DIR}/dsdlc_generated --target-language cpp ${DSDL_DIR}/uavcan)

add_definitions(
	-DCANARD_DSDL_CONFIG_LITTLE_ENDIAN=1
)

px4_add_module(
	MODULE drivers__uavcan_v1
	MAIN uavcan_v1
	COMPILE_FLAGS
		-Wno-error
		-DUINT32_C\(x\)=__UINT32_C\(x\)
	INCLUDES
		${LIBCANARD_DIR}/libcanard/
		${CMAKE_CURRENT_BINARY_DIR}/dsdlc_generated
	SRCS
		Uavcan.cpp
		Uavcan.hpp
		#socketcan.c
		#socketcan.h
		o1heap/o1heap.c
		o1heap/o1heap.h
		${LIBCANARD_DIR}/libcanard/canard_dsdl.c
		${LIBCANARD_DIR}/libcanard/canard_dsdl.h
		${LIBCANARD_DIR}/libcanard/canard.c
		${LIBCANARD_DIR}/libcanard/canard.h
	DEPENDS
		git_libcanard
		git_public_regulated_data_types
		version
	)
