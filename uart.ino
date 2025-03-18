#include "inc/Vec3.h"
#include "inc/DroneController.h"

// TODO: Find correct serial port
TeensyUART uart_manager(Serial1, 9600);

void ReceiveControlOutput(Payload &payload)
{
    ControlOutput output;
    bool success = true;

    success &= payload.ReadDouble(output.d1);
    success &= payload.ReadDouble(output.d2);
    success &= payload.ReadDouble(output.thrust);
    success &= payload.ReadDouble(output.mz);

    if (!success)
    {
        Serial.println("Error: Failed to read complete ControlOutput from payload!");
        return;
    }

    received_control_output = output;
    control_output_received = true;
}

bool SendControlInput(const ControlInput &input)
{
    Payload payload;
    bool success = true;

    // Write state
    success &= WriteVec3(payload, input.state.attitude);
    success &= WriteVec3(payload, input.state.rate);
    success &= payload.WriteInt(input.state.attitude_count);
    success &= payload.WriteInt(input.state.rate_count);

    // Write remote input
    success &= WriteVec3(payload, input.remote_input.att_ref);
    success &= payload.WriteDouble(input.remote_input.inline_thrust);
    success &= payload.WriteDouble(input.remote_input.yaw_rate_ref);
    success &= payload.WriteBool(input.remote_input.arm);

    if (!success)
    {
        Serial.println("Error: Failed to write control input to payload");
        return false;
    }

    success = uart_manager.SendUARTPacket((int)PacketId::ControlInput, payload);

    if (!success)
    {
        Serial.println("Error: Failed to send control input");
        return false;
    }

    return true;
}

bool WriteVec3(Payload &payload, Vec3 vec)
{
    bool success = true;
    success &= payload.WriteDouble(vec.x);
    success &= payload.WriteDouble(vec.y);
    success &= payload.WriteDouble(vec.z);

    if (!success)
    {
        Serial.println("Error: Failed to write Vec3 to payload");
        return false;
    }

    return true;
}

void setup_uart()
{
    uart_manager.Begin();
    // uart_manager.RegisterHandler((int)PacketId::ControlInput, ReceiveControlInput);
    uart_manager.RegisterHandler((int)PacketId::ControlOutput, ReceiveControlOutput);
}

void update_uart()
{
    uart_manager.ReceiveUARTPackets();
    uart_manager.SendUARTPackets();
}