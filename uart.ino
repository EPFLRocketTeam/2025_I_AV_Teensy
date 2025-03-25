TeensyUART uart_manager(Serial7, 115200);

void ReceiveControlOutput(Payload &payload)
{
    ControlOutputPacket output_packet;
    bool success = payload.ReadControlOutputPacket(output_packet);

    if (!success)
    {
        Serial.println("Error: Failed to read output packet from payload!");
        return;
    }

    // TODO: Average this value
    // TODO: Log this to SD card
    double round_trip_time = millis() - output_packet.timestamp;
    Serial.print("Round trip time: ");
    Serial.println(round_trip_time);

    // Serial.print("Received control output: d1=");
    // Serial.print(output.d1);
    // Serial.print(", d2=");
    // Serial.print(output.d2);
    // Serial.print(", thrust=");
    // Serial.print(output.thrust);
    // Serial.print(", mz=");
    // Serial.println(output.mz);

    received_output = {
        output_packet.d1,
        output_packet.d2,
        output_packet.avg_throttle,
        output_packet.throttle_diff
    };
}

bool SendControlInput(const ControlInputPacket &input)
{
    Payload payload;
    bool success = payload.WriteControlInputPacket(input);
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

    Serial.print("Sent control input: armed=");
    Serial.print(input.armed);
    Serial.print(", thrust=");
    Serial.print(input.inline_thrust);
    Serial.println(", rest_of_the_payload=TODO");

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