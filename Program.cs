using GPSReceiver;
using System;


using GPSSensor gps = new();

gps.Start("COM7");


Console.WriteLine("Press any key to stop.");
Console.ReadKey(true);

gps.Stop();
