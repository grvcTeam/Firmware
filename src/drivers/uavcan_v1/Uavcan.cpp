/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "Uavcan.hpp"

#include <lib/ecl/geo/geo.h>
#include <lib/version/version.h>

#include "stm32_can.h"
#include <nuttx/can/can.h>

#define REGULATED_DRONE_SENSOR_BMSSTATUS_ID 32080

using namespace time_literals;

UavcanNode *UavcanNode::_instance;

O1HeapInstance *uavcan_allocator{nullptr};

static void *memAllocate(CanardInstance *const ins, const size_t amount) { return o1heapAllocate(uavcan_allocator, amount); }
static void memFree(CanardInstance *const ins, void *const pointer) { o1heapFree(uavcan_allocator, pointer); }


UavcanNode::UavcanNode(uint32_t node_id) :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::uavcan)
{
	pthread_mutex_init(&_node_mutex, nullptr);

	_uavcan_heap = new uint8_t[HeapSize];
	uavcan_allocator = o1heapInit(&_uavcan_heap, HeapSize, nullptr, nullptr);

	if (uavcan_allocator == nullptr) {
		PX4_ERR("o1heapInit failed with size %d", HeapSize);
	}

	_canard_instance = canardInit(&memAllocate, &memFree);

	_canard_instance.node_id = node_id; // Defaults to anonymous; can be set up later at any point.

	bool can_fd = false;

	if (can_fd) {
		_canard_instance.mtu_bytes = CANARD_MTU_CAN_FD;

	} else {
		_canard_instance.mtu_bytes = CANARD_MTU_CAN_CLASSIC;
	}
}

UavcanNode::~UavcanNode()
{
	if (_instance) {
		/* tell the task we want it to go away */
		_task_should_exit.store(true);
		ScheduleNow();

		unsigned i = 10;

		do {
			/* wait 5ms - it should wake every 10ms or so worst-case */
			usleep(5000);

			if (--i == 0) {
				break;
			}

		} while (_instance);
	}

	perf_free(_cycle_perf);
	perf_free(_interval_perf);
}

int UavcanNode::start(uint32_t node_id, uint32_t bitrate)
{
	if (_instance != nullptr) {
		PX4_WARN("Already started");
		return -1;
	}

	_instance = new UavcanNode(node_id);

	if (_instance == nullptr) {
		PX4_ERR("Out of memory");
		return -1;
	}

	const int node_init_res = 0; //_instance->init(node_id, can->driver.updateEvent());

	if (node_init_res < 0) {
		delete _instance;
		_instance = nullptr;
		PX4_ERR("Node init failed %i", node_init_res);
		return node_init_res;
	}

	// Keep the bit rate for reboots on BenginFirmware updates
	_instance->active_bitrate = bitrate;

	_instance->ScheduleOnInterval(ScheduleIntervalMs * 1000);

	return PX4_OK;
}

void UavcanNode::busevent_signal_trampoline()
{
	if (_instance) {
		// trigger the work queue (Note, this is called from IRQ context)
		_instance->ScheduleNow();
	}
}

void UavcanNode::Run()
{
	pthread_mutex_lock(&_node_mutex);

	if (!_initialized) {

		struct can_dev_s *can = stm32_caninitialize(1);

		if (can == nullptr) {
			PX4_ERR("Failed to get CAN interface");

		} else {
			/* Register the CAN driver at "/dev/can0" */
			int ret = can_register("/dev/can0", can);

			if (ret < 0) {
				PX4_ERR("can_register failed: %d", ret);

			} else {
				_can_fd = ::open("/dev/can0", O_RDWR | O_NONBLOCK);
				_initialized = true;
			}
		}

		canardRxSubscribe(&_canard_instance,   // Subscribe to messages uavcan.node.Heartbeat.
				  CanardTransferKindMessage,
				  uavcan::node::Heartbeat_1_0::PORT_ID,  // The fixed Subject-ID of the Heartbeat message type (see DSDL definition).
				  uavcan::node::Heartbeat_1_0::SIZE,      // The maximum payload size (max DSDL object size) from the DSDL definition.
				  CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
				  &_heartbeat_subscription);


		canardRxSubscribe(&_canard_instance,   // Subscribe to messages uavcan.node.Heartbeat.
				  CanardTransferKindMessage,
				  REGULATED_DRONE_SENSOR_BMSSTATUS_ID,  // The fixed Subject-ID of the Heartbeat message type (see DSDL definition).
				  uavcan::node::Heartbeat_1_0::SIZE,      // The maximum payload size (max DSDL object size) from the DSDL definition.
				  CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
				  &_drone_sensor_BMSStatus_subscription);
	}

	perf_begin(_cycle_perf);
	perf_count(_interval_perf);

	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
	}

	// File desriptor for CAN.
	struct pollfd fds {};
	fds.fd = _can_fd;
	fds.events = POLLIN;

	// Any recieved CAN messages will cause the poll statement to unblock and run
	// This way CAN read runs with minimal latency.
	// Note that multiple messages may be received in a short time, so this will try to read any availible in a loop
	::poll(&fds, 1, 10);

	// Only execute this part if can0 is changed.
	if (fds.revents & POLLIN) {

		// Try to read.
		struct can_msg_s receive_msg;
		const ssize_t nbytes = ::read(fds.fd, &receive_msg, sizeof(receive_msg));

		if (nbytes < 0 || (size_t)nbytes < CAN_MSGLEN(0) || (size_t)nbytes > sizeof(receive_msg)) {
			// error
		} else {

			CanardFrame received_frame{};
			received_frame.extended_can_id = receive_msg.cm_hdr.ch_id;
			received_frame.payload_size = receive_msg.cm_hdr.ch_dlc;
			received_frame.payload = &receive_msg.cm_data;

			CanardTransfer receive;
			int32_t result = canardRxAccept(&_canard_instance, &received_frame, 0, &receive);

			if (result < 0) {
				// An error has occurred: either an argument is invalid or we've ran out of memory.
				// It is possible to statically prove that an out-of-memory will never occur for a given application if
				// the heap is sized correctly; for background, refer to the Robson's Proof and the documentation for O1Heap.
				// Reception of an invalid frame is NOT an error.
				PX4_ERR("Receive error %d\n", result);

			} else if (result == 1) {
				// A transfer has been received, process it.
				_canard_instance.memory_free(&_canard_instance, (void *)receive.payload); // Deallocate the dynamic memory afterwards.

				if (receive.port_id == REGULATED_DRONE_SENSOR_BMSSTATUS_ID) {

					auto bms_status = regulated::drone::sensor::BMSStatus_1_0::deserializeFromBuffer((const uint8_t *)receive.payload,
							  receive.payload_size);

					battery_status_s battery_status{};
					battery_status.id = bms_status.battery_id;
					battery_status.remaining = bms_status.remaining_capacity;
					battery_status.timestamp = hrt_absolute_time();
					_battery_status_pub.publish(battery_status);
				}

			} else {
				PX4_INFO("RX canard %d", result);
				// Nothing to do.
				// The received frame is either invalid or it's a non-last frame of a multi-frame transfer.
				// Reception of an invalid frame is NOT reported as an error because it is not an error.
			}
		}

	}


	perf_end(_cycle_perf);

	if (_task_should_exit.load()) {
		ScheduleClear();
		::close(_can_fd);
		_instance = nullptr;
	}

	pthread_mutex_unlock(&_node_mutex);
}

void UavcanNode::print_info()
{
	pthread_mutex_lock(&_node_mutex);

	perf_print_counter(_cycle_perf);
	perf_print_counter(_interval_perf);

	pthread_mutex_unlock(&_node_mutex);
}

static void print_usage()
{
	PX4_INFO("usage: \n"
		 "\tuavcannode {start|status|stop}");
}

extern "C" int uavcan_start(int argc, char *argv[])
{
	// CAN bitrate
	int32_t bitrate = 0;
	param_get(param_find("UAVCAN_BITRATE"), &bitrate);

	// Node ID
	int32_t node_id = 0;
	param_get(param_find("UAVCAN_NODE_ID"), &node_id);

	// Start
	PX4_INFO("Node ID %u, bitrate %u", node_id, bitrate);
	int rv = UavcanNode::start(node_id, bitrate);

	return rv;
}

extern "C" __EXPORT int uavcan_v1_main(int argc, char *argv[])
{
	if (argc < 2) {
		print_usage();
		return 1;
	}

	if (!strcmp(argv[1], "start")) {
		if (UavcanNode::instance()) {
			PX4_ERR("already started");
			return 1;
		}

		return uavcan_start(argc, argv);
	}

	/* commands below require the app to be started */
	UavcanNode *const inst = UavcanNode::instance();

	if (!inst) {
		PX4_ERR("application not running");
		return 1;
	}

	if (!strcmp(argv[1], "status") || !strcmp(argv[1], "info")) {
		inst->print_info();
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		delete inst;
		return 0;
	}

	print_usage();
	return 1;
}
