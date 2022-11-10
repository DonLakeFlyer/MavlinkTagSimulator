#pragma once

// DEBUG_FLOAT_ARRAY
//  time_usec - send index (used to detect lost telemetry)
//  array_id - command id

#define COMMAND_ID_ACK                          1   // Ack response to command
#define COMMAND_ID_TAG                          2   // Tag info
#define COMMAND_ID_START_DETECTION              3   // Start pulse detection
#define COMMAND_ID_STOP_DETECTION               4   // Stop pulse detection
#define COMMAND_ID_PULSE                        5   // Detected pulse value

#define ACK_IDX_COMMAND                         0   // Command being acked
#define ACK_IDX_RESULT                          1   // Command result - 1 success, 0 failure

#define PULSE_IDX_TIME_SECS						0	// Pulse time in seconds (double)
#define PULSE_IDX_CONFIRMED_STATUS              18 	// Confirmation status (bool)
#define PULSE_IDX_SNR                      		9   // Pulse SNR (float)
#define PULSE_IDX_GROUP_INDEX                   15  // Group index 0/1/2 (uint 32)

#define PULSE_DETECTION_STATUS_SUPER_THRESHOLD  1
#define PULSE_DETECTION_STATUS_CONFIRMED        2

#define TAG_IDX_ID                              0   // Tag id (uint 32)
#define TAG_IDX_FREQUENCY                       1   // Frequency - 6 digits shifted by three decimals, NNNNNN means NNN.NNN000 Mhz (uint 32)
#define TAG_IDX_DURATION_MSECS                  2   // Pulse duration
#define TAG_IDX_INTRA_PULSE1_MSECS              3   // Intra-pulse duration 1
#define TAG_IDX_INTRA_PULSE2_MSECS              4   // Intra-pulse duration 2
#define TAG_IDX_INTRA_PULSE_UNCERTAINTY         5   // Intra-pulse uncertainty
#define TAG_IDX_INTRA_PULSE_JITTER              6   // Intra-pulse jitter
#define TAG_IDX_MAX_PULSE                       7   // Max pulse value

#define START_DETECTION_IDX_TAG_ID              0   // Tag to start detection on

typedef struct {
	uint32_t command;
} HeaderInfo_t;

typedef struct {
	HeaderInfo_t 	header = { COMMAND_ID_ACK };

	uint32_t		command;
	uint32_t		result;
} AckInfo_t;

typedef struct {
	HeaderInfo_t	header = {COMMAND_ID_TAG};

	uint32_t		frequency;
	uint32_t		pulseWidthMSecs;
	uint32_t		intraPulse1MSecs;
	uint32_t		intraPulse2MSecs;
	uint32_t		intraPulseUncertaintyMsecs;
	uint32_t		intraPulseJitterMSecs;
} TagInfo_t;
