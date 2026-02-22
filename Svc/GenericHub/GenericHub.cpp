// ======================================================================
// \title  GenericHub.cpp
// \author mstarch
// \brief  cpp file for GenericHub component implementation class
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged.
//
// ======================================================================

#include <Fw/FPrimeBasicTypes.hpp>
#include <Svc/GenericHub/GenericHub.hpp>
#include <cstring>
#include "Fw/Logger/Logger.hpp"
#include "Fw/Types/Assert.hpp"

// Platform-specific byte order helpers
#ifdef __ZEPHYR__
#include <zephyr/sys/byteorder.h>
#define cpu_to_le16(x) sys_cpu_to_le16(x)
#define le16_to_cpu(x) sys_le16_to_cpu(x)
#else
#include <endian.h>
#define cpu_to_le16(x) htole16(x)
#define le16_to_cpu(x) le16toh(x)
#endif

// Required port serialization or the hub cannot work
static_assert(FW_PORT_SERIALIZATION, "FW_PORT_SERIALIZATION must be enabled to use GenericHub");

namespace Svc {

// Wire header: tightly packed to avoid padding
#pragma pack(push, 1)
struct HubHeader {
    U8 type;
    U8 port;
    U16 size;
};
#pragma pack(pop)

// ----------------------------------------------------------------------
// Construction, initialization, and destruction
// ----------------------------------------------------------------------

GenericHub::GenericHub(const char* const compName) : GenericHubComponentBase(compName) {}

GenericHub::~GenericHub() {}

void GenericHub::send_data(const HubType type, const FwIndexType port, const U8* data, const FwSizeType size) {
    if (data == nullptr) {
        Fw::Logger::log("GenericHub: send_data nullptr payload");
        return;
    }
    if (size == 0) {
        Fw::Logger::log("GenericHub: send_data zero-length payload");
        return;
    }

    const FwSizeType totalSize = static_cast<FwSizeType>(sizeof(HubHeader) + size);
    Fw::Buffer outgoing = allocate_out(0, static_cast<U32>(totalSize));

    // Build packed header with little-endian fields
    HubHeader header = {};
    header.type = static_cast<U8>(type);
    header.port = static_cast<U8>(port);
    header.size = cpu_to_le16(static_cast<U16>(size));

    // Copy header + payload
    U8* outData = outgoing.getData();
    memcpy(outData, &header, sizeof(header));
    memcpy(outData + sizeof(header), data, size);

    outgoing.setSize(static_cast<U32>(totalSize));

    toBufferDriver_out(0, outgoing);
}

// ----------------------------------------------------------------------
// Handler implementations for user-defined typed input ports
// ----------------------------------------------------------------------

void GenericHub::bufferIn_handler(const FwIndexType portNum, Fw::Buffer& fwBuffer) {
    send_data(HUB_TYPE_BUFFER, portNum, fwBuffer.getData(), fwBuffer.getSize());
    bufferInReturn_out(portNum, fwBuffer);
}

void GenericHub::bufferOutReturn_handler(FwIndexType portNum, Fw::Buffer& fwBuffer) {
    // Return the buffer
    fromBufferDriverReturn_out(0, fwBuffer);
}

void GenericHub::fromBufferDriver_handler(const FwIndexType portNum, Fw::Buffer& fwBuffer) {
    if (fwBuffer.getSize() == 0 || fwBuffer.getData() == nullptr) {
        fromBufferDriverReturn_out(0, fwBuffer);
        return;
    }

    // Append incoming data to accumulator
    if (this->m_accumulatorSize + fwBuffer.getSize() > sizeof(this->m_accumulator)) {
        // Buffer overflow - clear accumulator and log error
        Fw::Logger::log("[GenericHub] Accumulator OVERFLOW! Resetting...\n");
        this->m_accumulatorSize = 0;
    }
    memcpy(this->m_accumulator + this->m_accumulatorSize, fwBuffer.getData(), fwBuffer.getSize());
    this->m_accumulatorSize += fwBuffer.getSize();

    // Process all complete frames in accumulator
    while (this->m_accumulatorSize >= sizeof(HubHeader)) {
        // Parse header
        HubHeader header;
        memcpy(&header, this->m_accumulator, sizeof(HubHeader));
        const FwBuffSizeType payloadSize = le16_to_cpu(header.size);
        const U32 frameSize = sizeof(HubHeader) + payloadSize;

        if (this->m_accumulatorSize < frameSize) {
            // Wait for more data
            break;
        }

        // We have a full frame! Process it
        const HubType type = static_cast<HubType>(header.type);
        const U32 port = header.port;
        U8* payload = this->m_accumulator + sizeof(HubHeader);

        // --- Process Frame Logic (same as before) ---
        if (type == HUB_TYPE_PORT) {
            if (port < this->getNum_serialOut_OutputPorts()) {
                Fw::ExternalSerializeBuffer wrapper(payload, payloadSize);
                wrapper.setBuffLen(payloadSize);
                serialOut_out(static_cast<FwIndexType>(port), wrapper);
            }
        } else if (type == HUB_TYPE_BUFFER) {
            if (port < this->getNum_bufferOut_OutputPorts()) {
                // Buffer needs special handling - we must allocate a new buffer since we can't pass accumulator pointer
                Fw::Buffer newBuf = allocate_out(0, payloadSize);
                if (newBuf.getData() != nullptr) {
                    memcpy(newBuf.getData(), payload, payloadSize);
                    newBuf.setSize(payloadSize);
                    bufferOut_out(static_cast<FwIndexType>(port), newBuf);
                }
            }
        } else if (type == HUB_TYPE_EVENT) {
            if (port < this->getNum_eventOut_OutputPorts()) {
                FwEventIdType id;
                Fw::Time timeTag;
                Fw::LogSeverity severity;
                Fw::LogBuffer args;
                Fw::ExternalSerializeBuffer deserializer(payload, payloadSize);
                deserializer.setBuffLen(payloadSize);
                if (deserializer.deserializeTo(id) == Fw::FW_SERIALIZE_OK &&
                    deserializer.deserializeTo(timeTag) == Fw::FW_SERIALIZE_OK &&
                    deserializer.deserializeTo(severity) == Fw::FW_SERIALIZE_OK &&
                    deserializer.deserializeTo(args) == Fw::FW_SERIALIZE_OK) {
                    this->eventOut_out(static_cast<FwIndexType>(port), id, timeTag, severity, args);
                }
            }
        } else if (type == HUB_TYPE_CHANNEL) {
            if (port < this->getNum_tlmOut_OutputPorts()) {
                FwChanIdType id;
                Fw::Time timeTag;
                Fw::TlmBuffer val;
                Fw::ExternalSerializeBuffer deserializer(payload, payloadSize);
                deserializer.setBuffLen(payloadSize);
                if (deserializer.deserializeTo(id) == Fw::FW_SERIALIZE_OK &&
                    deserializer.deserializeTo(timeTag) == Fw::FW_SERIALIZE_OK &&
                    deserializer.deserializeTo(val) == Fw::FW_SERIALIZE_OK) {
                    this->tlmOut_out(static_cast<FwIndexType>(port), id, timeTag, val);
                }
            }
        }

        // Shift accumulator
        const U32 remaining = this->m_accumulatorSize - frameSize;
        if (remaining > 0) {
            memmove(this->m_accumulator, this->m_accumulator + frameSize, remaining);
        }
        this->m_accumulatorSize = remaining;
    }

    // Always return the driver buffer
    fromBufferDriverReturn_out(0, fwBuffer);
}

void GenericHub::toBufferDriverReturn_handler(FwIndexType portNum, Fw::Buffer& fwBuffer) {
    // Deallocate the existing buffer
    deallocate_out(portNum, fwBuffer);
}

void GenericHub::eventIn_handler(const FwIndexType portNum,
                                 FwEventIdType id,
                                 Fw::Time& timeTag,
                                 const Fw::LogSeverity& severity,
                                 Fw::LogBuffer& args) {
    Fw::SerializeStatus status = Fw::FW_SERIALIZE_OK;
    U8 buffer[sizeof(FwEventIdType) + Fw::Time::SERIALIZED_SIZE + Fw::LogSeverity::SERIALIZED_SIZE +
              FW_LOG_BUFFER_MAX_SIZE];
    Fw::ExternalSerializeBuffer serializer(buffer, sizeof(buffer));
    serializer.resetSer();
    status = serializer.serializeFrom(id);
    FW_ASSERT(status == Fw::SerializeStatus::FW_SERIALIZE_OK);
    status = serializer.serializeFrom(timeTag);
    FW_ASSERT(status == Fw::SerializeStatus::FW_SERIALIZE_OK);
    status = serializer.serializeFrom(severity);
    FW_ASSERT(status == Fw::SerializeStatus::FW_SERIALIZE_OK);
    status = serializer.serializeFrom(args);
    FW_ASSERT(status == Fw::SerializeStatus::FW_SERIALIZE_OK);
    FwSizeType size = serializer.getSize();
    this->send_data(HubType::HUB_TYPE_EVENT, portNum, buffer, size);
}

void GenericHub::tlmIn_handler(const FwIndexType portNum, FwChanIdType id, Fw::Time& timeTag, Fw::TlmBuffer& val) {
    Fw::SerializeStatus status = Fw::FW_SERIALIZE_OK;
    U8 buffer[sizeof(FwChanIdType) + Fw::Time::SERIALIZED_SIZE + FW_TLM_BUFFER_MAX_SIZE];
    Fw::ExternalSerializeBuffer serializer(buffer, sizeof(buffer));
    serializer.resetSer();
    status = serializer.serializeFrom(id);
    FW_ASSERT(status == Fw::SerializeStatus::FW_SERIALIZE_OK);
    status = serializer.serializeFrom(timeTag);
    FW_ASSERT(status == Fw::SerializeStatus::FW_SERIALIZE_OK);
    status = serializer.serializeFrom(val);
    FW_ASSERT(status == Fw::SerializeStatus::FW_SERIALIZE_OK);
    FwSizeType size = serializer.getSize();
    this->send_data(HubType::HUB_TYPE_CHANNEL, portNum, buffer, size);
}

// ----------------------------------------------------------------------
// Handler implementations for user-defined serial input ports
// ----------------------------------------------------------------------

void GenericHub::serialIn_handler(FwIndexType portNum,            /*!< The port number*/
                                  Fw::SerializeBufferBase& Buffer /*!< The serialization buffer*/
) {
    send_data(HUB_TYPE_PORT, portNum, Buffer.getBuffAddr(), Buffer.getSize());
}

}  // end namespace Svc
