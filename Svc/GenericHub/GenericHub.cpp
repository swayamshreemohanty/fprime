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
#include "Fw/Logger/Logger.hpp"
#include "Fw/Types/Assert.hpp"
#include <cstring>

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
    static uint32_t rxFrameCount = 0;

    if (fwBuffer.getSize() < sizeof(HubHeader)) {
        rxFrameCount++;
        fromBufferDriverReturn_out(0, fwBuffer);
        return;
    }

    // Parse packed header with little-endian fields
    HubHeader header = {};
    memcpy(&header, fwBuffer.getData(), sizeof(HubHeader));

    const HubType type = static_cast<HubType>(header.type);
    if (type >= HUB_TYPE_MAX) {
        rxFrameCount++;
        fromBufferDriverReturn_out(0, fwBuffer);
        return;
    }

    const U32 port = header.port;
    const FwBuffSizeType size = le16_to_cpu(header.size);
    
    // Validate size matches buffer
    U8* rawData = fwBuffer.getData() + sizeof(HubHeader);
    const U32 rawSize = static_cast<U32>(fwBuffer.getSize() - sizeof(HubHeader));
    Fw::SerializeStatus status = Fw::FW_SERIALIZE_OK;

    if (rawSize != static_cast<U32>(size)) {
        rxFrameCount++;
        fromBufferDriverReturn_out(0, fwBuffer);
        return;
    }
    
    rxFrameCount++;
    if (type == HUB_TYPE_PORT) {
        if (port >= this->getNum_serialOut_OutputPorts()) {
            Fw::Logger::log("[GenericHub] ERROR: invalid serial port %u (max %u)", port, this->getNum_serialOut_OutputPorts());
            fromBufferDriverReturn_out(0, fwBuffer);
            return;
        }
        // Com buffer representations should be copied before the call returns, so we need not "allocate" new data
        Fw::ExternalSerializeBuffer wrapper(rawData, rawSize);
        status = wrapper.setBuffLen(rawSize);
        if (status != Fw::FW_SERIALIZE_OK) {
            Fw::Logger::log("[GenericHub] ERROR: wrapper.setBuffLen failed %d", status);
            fromBufferDriverReturn_out(0, fwBuffer);
            return;
        }
        Fw::Logger::log("[GenericHub] → Forwarding PORT to serialOut[%u]", port);
        serialOut_out(static_cast<FwIndexType>(port), wrapper);
        // Deallocate the existing buffer
        fromBufferDriverReturn_out(0, fwBuffer);
    } else if (type == HUB_TYPE_BUFFER) {
        if (port >= this->getNum_bufferOut_OutputPorts()) {
            Fw::Logger::log("[GenericHub] ERROR: invalid buffer port %u (max %u)", port, this->getNum_bufferOut_OutputPorts());
            fromBufferDriverReturn_out(0, fwBuffer);
            return;
        }
        Fw::Logger::log("[GenericHub] → Forwarding BUFFER to bufferOut[%u]", port);
        // Fw::Buffers can reuse the existing data buffer as the storage type!  No deallocation done.
        fwBuffer.set(rawData, rawSize, fwBuffer.getContext());
        bufferOut_out(static_cast<FwIndexType>(port), fwBuffer);
    } else if (type == HUB_TYPE_EVENT) {
        if (port >= this->getNum_eventOut_OutputPorts()) {
            Fw::Logger::log("[GenericHub] ERROR: invalid event port %u (max %u)", port, this->getNum_eventOut_OutputPorts());
            fromBufferDriverReturn_out(0, fwBuffer);
            return;
        }
        Fw::Logger::log("[GenericHub] Processing EVENT packet, port=%u", port);
        FwEventIdType id;
        Fw::Time timeTag;
        Fw::LogSeverity severity;
        Fw::LogBuffer args;

        // Deserialize tokens for events from the payload
        Fw::ExternalSerializeBuffer deserializer(rawData, rawSize);
        Fw::SerializeStatus status = deserializer.setBuffLen(rawSize);
        FW_ASSERT(status == Fw::FW_SERIALIZE_OK, static_cast<FwAssertArgType>(status));
        
        status = deserializer.deserializeTo(id);
        FW_ASSERT(status == Fw::FW_SERIALIZE_OK, static_cast<FwAssertArgType>(status));
        status = deserializer.deserializeTo(timeTag);
        FW_ASSERT(status == Fw::FW_SERIALIZE_OK, static_cast<FwAssertArgType>(status));
        status = deserializer.deserializeTo(severity);
        FW_ASSERT(status == Fw::FW_SERIALIZE_OK, static_cast<FwAssertArgType>(status));
        status = deserializer.deserializeTo(args);
        FW_ASSERT(status == Fw::FW_SERIALIZE_OK, static_cast<FwAssertArgType>(status));

        // Send it!
        this->eventOut_out(static_cast<FwIndexType>(port), id, timeTag, severity, args);

        // Deallocate the existing buffer
        fromBufferDriverReturn_out(0, fwBuffer);
    } else if (type == HUB_TYPE_CHANNEL) {
        if (port >= this->getNum_tlmOut_OutputPorts()) {
            Fw::Logger::log("[GenericHub] ERROR: invalid tlm port %u (max %u)", port, this->getNum_tlmOut_OutputPorts());
            fromBufferDriverReturn_out(0, fwBuffer);
            return;
        }
        FwChanIdType id;
        Fw::Time timeTag;
        Fw::TlmBuffer val;

        // Deserialize tokens for channels from the payload
        Fw::ExternalSerializeBuffer deserializer(rawData, rawSize);
        Fw::SerializeStatus status = deserializer.setBuffLen(rawSize);
        FW_ASSERT(status == Fw::FW_SERIALIZE_OK, static_cast<FwAssertArgType>(status));
        
        status = deserializer.deserializeTo(id);
        FW_ASSERT(status == Fw::FW_SERIALIZE_OK, static_cast<FwAssertArgType>(status));
        status = deserializer.deserializeTo(timeTag);
        FW_ASSERT(status == Fw::FW_SERIALIZE_OK, static_cast<FwAssertArgType>(status));
        status = deserializer.deserializeTo(val);
        FW_ASSERT(status == Fw::FW_SERIALIZE_OK, static_cast<FwAssertArgType>(status));

        // Send it!
        this->tlmOut_out(static_cast<FwIndexType>(port), id, timeTag, val);

        // Return the received buffer
        fromBufferDriverReturn_out(0, fwBuffer);
    } else {
        // Unknown type should already be filtered, but return buffer defensively
        fromBufferDriverReturn_out(0, fwBuffer);
    }
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
