#pragma once

#include <Arduino.h>
#include <stdio.h>
#include <string.h>

enum PacketType {
	PACKET_INVALID = 0,
	PACKET_SOS,
	PACKET_ACK,
	PACKET_CALL,
};

struct ProtocolMessage {
	PacketType type;
	uint32_t deviceId;
	uint32_t sequence;
};

static const size_t PROTOCOL_MAX_PAYLOAD = 48;

inline const char *protocolTypeName(PacketType type) {
	switch (type) {
	case PACKET_SOS:
		return "SOS";
	case PACKET_ACK:
		return "ACK";
	case PACKET_CALL:
		return "CALL";
	default:
		return "INVALID";
	}
}

inline bool protocolFormatMessage(const ProtocolMessage &message, char *buffer, size_t bufferSize) {
	int written = snprintf(
		buffer,
		bufferSize,
		"%s,%lu,%lu",
		protocolTypeName(message.type),
		(unsigned long)message.deviceId,
		(unsigned long)message.sequence);

	return written > 0 && (size_t)written < bufferSize;
}

inline bool protocolParseMessage(const uint8_t *data, uint16_t length, ProtocolMessage &message) {
	if (length == 0 || length >= PROTOCOL_MAX_PAYLOAD) {
		return false;
	}

	char payload[PROTOCOL_MAX_PAYLOAD];
	memcpy(payload, data, length);
	payload[length] = '\0';

	char typeBuffer[8];
	unsigned long deviceId = 0;
	unsigned long sequence = 0;

	if (sscanf(payload, "%7[^,],%lu,%lu", typeBuffer, &deviceId, &sequence) != 3) {
		return false;
	}

	if (strcmp(typeBuffer, "SOS") == 0) {
		message.type = PACKET_SOS;
	} else if (strcmp(typeBuffer, "ACK") == 0) {
		message.type = PACKET_ACK;
	} else if (strcmp(typeBuffer, "CALL") == 0) {
		message.type = PACKET_CALL;
	} else {
		message.type = PACKET_INVALID;
		return false;
	}

	message.deviceId = (uint32_t)deviceId;
	message.sequence = (uint32_t)sequence;
	return true;
}

inline bool protocolMatchesResponse(const ProtocolMessage &response, PacketType expectedType, uint32_t deviceId, uint32_t sequence) {
	return response.type == expectedType && response.deviceId == deviceId && response.sequence == sequence;
}