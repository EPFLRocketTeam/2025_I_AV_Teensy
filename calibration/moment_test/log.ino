void double_print(String s)
{
    Serial.print(s);
    flightFile.print(s);
}

void print_cols()
{
    double_print("time,avg_throttle,throttle_diff,z_rate,manual,stabilising");
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


void print_state(double time, double avg_throttle, double throttle_diff, double z_rate, bool manual, bool stabilising)
{
    std::list<double> toPrint = {time, avg_throttle, throttle_diff, z_rate, (double) manual, (double) stabilising};
    print_line(toPrint);
}
