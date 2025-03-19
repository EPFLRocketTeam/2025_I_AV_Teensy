TeensyUART uart_manager(Serial7, 115200);

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

    Serial.print("Received control output: d1=");
    Serial.print(output.d1);
    Serial.print(", d2=");
    Serial.print(output.d2);
    Serial.print(", thrust=");
    Serial.print(output.thrust);
    Serial.print(", mz=");
    Serial.println(output.mz);

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

    Serial.print("Sent control input: att_count=");
    Serial.print(input.state.attitude_count);
    Serial.print(", rate_count=");
    Serial.print(input.state.rate_count);
    Serial.print(", inline_thrust=");
    Serial.print(input.remote_input.inline_thrust);
    Serial.print(", yaw_rate_ref=");
    Serial.print(input.remote_input.yaw_rate_ref);
    Serial.print(", arm=");
    Serial.println(input.remote_input.arm);

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