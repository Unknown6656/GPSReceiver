using GPSReceiver;
using System;


using GPSSensor gps = new();

gps.OnValidDataReceived += (_, d) =>
{
    if (d is NMEA0183Data.GNSSData { Coordinates: var coord })
    {
        string s = coord.ToString();

        Console.Write($"> {s}{new string(' ', Console.WindowWidth - 3 - s.Length)}\r");
    }
};
//gps.OnRawDataReceived += (_, s) => Console.WriteLine(s);
gps.Start("COM7");


Console.WriteLine("Press any key to stop.");
Console.ReadKey(true);

gps.Stop();
