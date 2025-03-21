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

    success &= payload.WriteBool(input.armed);
    success &= WriteState(payload, input.desired_state);
    success &= WriteState(payload, input.current_state);
    success &= WriteSetpointSelection(payload, input.setpointSelection);
    success &= payload.WriteDouble(input.inline_thrust);
    
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

    Serial.print("Received control input: thrust=");
    Serial.print(input.inline_thrust);
    Serial.println(", rest_of_the_payload=TODO");
    
    return true;
}

bool WriteSetpointSelection(Payload &payload, const SetpointSelection &setpoint)
{
    // Pack all 12 boolean values into 2 bytes to minimize payload size
    uint8_t buffer[2] = {0, 0};

    // Pack posSPActive (3 bits)
    if (setpoint.posSPActive[0])
        buffer[0] |= (1 << 0);
    if (setpoint.posSPActive[1])
        buffer[0] |= (1 << 1);
    if (setpoint.posSPActive[2])
        buffer[0] |= (1 << 2);

    // Pack velSPActive (3 bits)
    if (setpoint.velSPActive[0])
        buffer[0] |= (1 << 3);
    if (setpoint.velSPActive[1])
        buffer[0] |= (1 << 4);
    if (setpoint.velSPActive[2])
        buffer[0] |= (1 << 5);

    // Pack attSPActive (3 bits across byte boundary)
    if (setpoint.attSPActive[0])
        buffer[0] |= (1 << 6);
    if (setpoint.attSPActive[1])
        buffer[0] |= (1 << 7);
    if (setpoint.attSPActive[2])
        buffer[1] |= (1 << 0);

    // Pack rateSPActive (3 bits)
    if (setpoint.rateSPActive[0])
        buffer[1] |= (1 << 1);
    if (setpoint.rateSPActive[1])
        buffer[1] |= (1 << 2);
    if (setpoint.rateSPActive[2])
        buffer[1] |= (1 << 3);

    // Write the packed bytes to the payload
    bool success = payload.WriteBytes(buffer, sizeof(buffer));

    if (!success)
    {
        Serial.println("Error: Failed to write SetpointSelection to payload");
        return false;
    }

    return true;
}

bool WriteState(Payload &payload, const State &state)
{
    bool success = true;
    success &= WriteVec3(payload, state.pos);
    success &= WriteVec3(payload, state.vel);
    success &= WriteVec3(payload, state.att);
    success &= WriteVec3(payload, state.rate);

    if (!success)
    {
        Serial.println("Error: Failed to write state to payload");
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
    uart_manager.RegisterHandler((int)PacketId::ControlOutput, ReceiveControlOutput);

    Serial.println("Finished UART setup");
}

void update_uart()
{
    uart_manager.ReceiveUARTPackets();
    uart_manager.SendUARTPackets();
}