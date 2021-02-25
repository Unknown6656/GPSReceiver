using GPSReceiver;
using System;


using GPSSensor gps = new();

gps.OnValidDataReceived += (_, d) =>
{
    if (d is NMEA0183Data.GNSSData { Coordinates: var coord })
        Console.WriteLine(coord);
};
gps.Start("COM7");


Console.WriteLine("Press any key to stop.");
Console.ReadKey(true);

gps.Stop();
