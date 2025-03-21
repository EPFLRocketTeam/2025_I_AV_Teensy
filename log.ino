void double_print(String s)
{
    Serial.print(s);
    flightFile.print(s);
}

void print_cols()
{
    double_print("time,d1,d2,throttle_avg,throttle_diff,att_count,rate_count,pressure_count,");

    for (String layer : {"rate", "attitude", "velocity", "position"})
    {
        for (String state : {"sp", "val", "output", "error", "pcont", "icont", "dcont"})
        {
            for (String dir : {"x", "y", "z"})
            {
                double_print(layer + "_" + dir + "_" + state + ",");
            }
        }
    }
    double_print("inline_thrust");
    double_print("\n");
}

void print_line(const std::list<double> &elements)
{
    String toPrint = "";
    int n = elements.size();
    int i = 0;
    for (double e : elements)
    {
        toPrint.append(String(e, 10));
        if (i != n - 1)
        {
            toPrint.append(",");
        }
        i++;
    }
    toPrint.append("\n");

    double_print(toPrint);
}


void print_state(double time, RawOutput out, const SensorData& sensor_data)
{
    std::list<double> toPrint = {time, out.d1, out.d2, out.avg_throttle, out.throttle_diff, (double) sensor_data.attitude_count, (double) sensor_data.rate_count, (double) sensor_data.pressure_count};
    // toPrint.splice(toPrint.end(), my_controller->getState());
    print_line(toPrint);
}

void print_vec(Vec3 v)
{
    print_line({v.x, v.y, v.z});
}

