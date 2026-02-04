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
#include <cstdio>

// Required port serialization or the hub cannot work
static_assert(FW_PORT_SERIALIZATION, "FW_PORT_SERIALIZATION must be enabled to use GenericHub");

namespace Svc {

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
    Fw::SerializeStatus status;
    // Buffer to send and a buffer used to write to it
    Fw::Buffer outgoing = allocate_out(0, static_cast<U32>(size + sizeof(U32) + sizeof(U32) + sizeof(FwBuffSizeType)));
    auto serialize = outgoing.getSerializer();
    // Write data to our buffer
    status = serialize.serializeFrom(static_cast<U32>(type));
    if (status != Fw::FW_SERIALIZE_OK) {
        Fw::Logger::log("GenericHub: serialize type failed %d", status);
        deallocate_out(0, outgoing);
        return;
    }
    status = serialize.serializeFrom(static_cast<U32>(port));
    if (status != Fw::FW_SERIALIZE_OK) {
        Fw::Logger::log("GenericHub: serialize port failed %d", status);
        deallocate_out(0, outgoing);
        return;
    }
    status = serialize.serializeFrom(data, size);
    if (status != Fw::FW_SERIALIZE_OK) {
        Fw::Logger::log("GenericHub: serialize payload failed %d", status);
        deallocate_out(0, outgoing);
        return;
    }
    outgoing.setSize(static_cast<U32>(serialize.getSize()));
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
    HubType type = HUB_TYPE_MAX;
    U32 type_in = 0;
    U32 port = 0;
    FwBuffSizeType size = 0;
    Fw::SerializeStatus status = Fw::FW_SERIALIZE_OK;
    const U32 headerSize = sizeof(U32) + sizeof(U32) + sizeof(FwBuffSizeType);

    // Drop obviously invalid packets before touching the buffer contents
    if (fwBuffer.getSize() < headerSize) {
        printf("[GenericHub] Dropped: size %u < header %u\n", fwBuffer.getSize(), headerSize);
        fflush(stdout);
        fromBufferDriverReturn_out(0, fwBuffer);
        return;
    }

    printf("[GenericHub] RX from STM32: size=%u bytes\n", fwBuffer.getSize());
    
    // Hex dump first 20 bytes for debugging
    U8* data = fwBuffer.getData();
    printf("[GenericHub]   Hex: ");
    for (U32 i = 0; i < fwBuffer.getSize() && i < 20; i++) {
        printf("%02X ", data[i]);
    }
    printf("\n");
    fflush(stdout);

    // Representation of incoming data prepped for serialization
    auto incoming = fwBuffer.getDeserializer();
    status = incoming.deserializeTo(type_in);
    if (status != Fw::FW_SERIALIZE_OK) {
        printf("[GenericHub] ERROR: deserialize type failed %d\n", status);
        fflush(stdout);
        fromBufferDriverReturn_out(0, fwBuffer);
        return;
    }
    printf("[GenericHub]   type_in=%u ", type_in);
    fflush(stdout);
    
    type = static_cast<HubType>(type_in);
    if (type >= HUB_TYPE_MAX) {
        printf("[GenericHub] ERROR: invalid type %u >= max %u\n", type_in, HUB_TYPE_MAX);
        fflush(stdout);
        fromBufferDriverReturn_out(0, fwBuffer);
        return;
    }
    status = incoming.deserializeTo(port);
    if (status != Fw::FW_SERIALIZE_OK) {
        printf("[GenericHub] ERROR: deserialize port failed %d\n", status);
        fflush(stdout);
        fromBufferDriverReturn_out(0, fwBuffer);
        return;
    }
    printf("port=%u ", port);
    fflush(stdout);
    
    status = incoming.deserializeTo(size);
    if (status != Fw::FW_SERIALIZE_OK) {
        printf("[GenericHub] ERROR: deserialize size failed %d\n", status);
        fflush(stdout);
        fromBufferDriverReturn_out(0, fwBuffer);
        return;
    }
    printf("size=%u\n", size);
    fflush(stdout);

    // Ensure payload size matches buffer size to avoid overruns on corrupt packets
    const U32 requiredSize = headerSize + static_cast<U32>(size);
    if (fwBuffer.getSize() < requiredSize) {
        printf("[GenericHub] ERROR: size mismatch buffer=%u < required=%u\n", fwBuffer.getSize(), requiredSize);
        fflush(stdout);
        fromBufferDriverReturn_out(0, fwBuffer);
        return;
    }
    if (size == 0) {
        printf("[GenericHub] ERROR: zero payload size\n");
        fflush(stdout);
        fromBufferDriverReturn_out(0, fwBuffer);
        return;
    }

    // Log packet type
    const char* typeNames[] = {"PORT", "BUFFER", "EVENT", "CHANNEL", "MAX"};
    printf("[GenericHub] Packet: type=%s(%u), port=%u, size=%u\n", 
           type < HUB_TYPE_MAX ? typeNames[type] : "UNKNOWN", type, port, size);
    fflush(stdout);

    // invokeSerial deserializes arguments before calling a normal invoke, this will return ownership immediately
    U8* rawData = fwBuffer.getData() + headerSize;
    U32 rawSize = static_cast<U32>(size);  // use declared payload size
    if (type == HUB_TYPE_PORT) {
        if (port >= this->getNum_serialOut_OutputPorts()) {
            printf("[GenericHub] ERROR: invalid serial port %u (max %u)\n", port, this->getNum_serialOut_OutputPorts());
            fflush(stdout);
            fromBufferDriverReturn_out(0, fwBuffer);
            return;
        }
        // Com buffer representations should be copied before the call returns, so we need not "allocate" new data
        Fw::ExternalSerializeBuffer wrapper(rawData, rawSize);
        status = wrapper.setBuffLen(rawSize);
        if (status != Fw::FW_SERIALIZE_OK) {
            printf("[GenericHub] ERROR: wrapper.setBuffLen failed %d\n", status);
            fflush(stdout);
            fromBufferDriverReturn_out(0, fwBuffer);
            return;
        }
        printf("[GenericHub] → Forwarding PORT to serialOut[%u]\n", port);
        fflush(stdout);
        serialOut_out(static_cast<FwIndexType>(port), wrapper);
        // Deallocate the existing buffer
        fromBufferDriverReturn_out(0, fwBuffer);
    } else if (type == HUB_TYPE_BUFFER) {
        if (port >= this->getNum_bufferOut_OutputPorts()) {
            printf("[GenericHub] ERROR: invalid buffer port %u (max %u)\n", port, this->getNum_bufferOut_OutputPorts());
            fflush(stdout);
            fromBufferDriverReturn_out(0, fwBuffer);
            return;
        }
        printf("[GenericHub] → Forwarding BUFFER to bufferOut[%u]\n", port);
        fflush(stdout);
        // Fw::Buffers can reuse the existing data buffer as the storage type!  No deallocation done.
        fwBuffer.set(rawData, rawSize, fwBuffer.getContext());
        bufferOut_out(static_cast<FwIndexType>(port), fwBuffer);
    } else if (type == HUB_TYPE_EVENT) {
        if (port >= this->getNum_eventOut_OutputPorts()) {
            printf("[GenericHub] ERROR: invalid event port %u (max %u)\n", port, this->getNum_eventOut_OutputPorts());
            fflush(stdout);
            fromBufferDriverReturn_out(0, fwBuffer);
            return;
        }
        printf("[GenericHub] Processing EVENT packet, port=%u\n", port);
        fflush(stdout);
        FwEventIdType id;
        Fw::Time timeTag;
        Fw::LogSeverity severity;
        Fw::LogBuffer args;

        // Deserialize tokens for events
        status = incoming.deserializeTo(id);
        if (status != Fw::FW_SERIALIZE_OK) {
            Fw::Logger::log("GenericHub: event id deserialize failed %d", status);
            fromBufferDriverReturn_out(0, fwBuffer);
            return;
        }
        status = incoming.deserializeTo(timeTag);
        if (status != Fw::FW_SERIALIZE_OK) {
            Fw::Logger::log("GenericHub: event time deserialize failed %d", status);
            fromBufferDriverReturn_out(0, fwBuffer);
            return;
        }
        status = incoming.deserializeTo(severity);
        if (status != Fw::FW_SERIALIZE_OK) {
            Fw::Logger::log("GenericHub: event severity deserialize failed %d", status);
            fromBufferDriverReturn_out(0, fwBuffer);
            return;
        }
        status = incoming.deserializeTo(args);
        if (status != Fw::FW_SERIALIZE_OK) {
            Fw::Logger::log("GenericHub: event args deserialize failed %d", status);
            fromBufferDriverReturn_out(0, fwBuffer);
            return;
        }

        // Send it!
        Fw::Logger::log("[GenericHub] Forwarding EVENT id=0x%x to eventOut[%u], severity=%d\n", id, port, severity.e);
        this->eventOut_out(static_cast<FwIndexType>(port), id, timeTag, severity, args);

        // Deallocate the existing buffer
        fromBufferDriverReturn_out(0, fwBuffer);
    } else if (type == HUB_TYPE_CHANNEL) {
        if (port >= this->getNum_tlmOut_OutputPorts()) {
            Fw::Logger::log("GenericHub: invalid tlm port %u (max %u)", port, this->getNum_tlmOut_OutputPorts());
            fromBufferDriverReturn_out(0, fwBuffer);
            return;
        }        printf("[GenericHub] Processing CHANNEL (telemetry) packet, port=%u\n", port);
        fflush(stdout);        FwChanIdType id;
        Fw::Time timeTag;
        Fw::TlmBuffer val;

        // Deserialize tokens for channels
        status = incoming.deserializeTo(id);
        if (status != Fw::FW_SERIALIZE_OK) {
            Fw::Logger::log("GenericHub: channel id deserialize failed %d", status);
            fromBufferDriverReturn_out(0, fwBuffer);
            return;
        }
        status = incoming.deserializeTo(timeTag);
        if (status != Fw::FW_SERIALIZE_OK) {
            Fw::Logger::log("GenericHub: channel time deserialize failed %d", status);
            fromBufferDriverReturn_out(0, fwBuffer);
            return;
        }
        status = incoming.deserializeTo(val);
        if (status != Fw::FW_SERIALIZE_OK) {
            Fw::Logger::log("GenericHub: channel val deserialize failed %d", status);
            fromBufferDriverReturn_out(0, fwBuffer);
            return;
        }

        // Send it!
        Fw::Logger::log("[GenericHub] Forwarding TELEMETRY id=0x%x to tlmOut[%u]\n", id, port);
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
