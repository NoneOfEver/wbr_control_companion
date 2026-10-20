/* Generated from chassismotors_feedback_raw.msg. Do not edit. */
#pragma once

#include <cstdint>
#include <cstddef>


namespace msg {

struct ChassisMotorFeedbackRawFrame {
	uint64_t timestamp_us;
	uint64_t precise_timestamp_us;
	bool valid;
	uint8_t data[8];
};


} // namespace msg
