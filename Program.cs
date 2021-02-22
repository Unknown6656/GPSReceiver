using System.IO.Ports;
using System;

namespace GPSReceiver
{
    public static class Program
    {
        public static void Main(string[] args)
        {
            using SerialPort port = new SerialPort("COM7", 9600)
            {
                DataBits = 8,
                Parity = Parity.None,
                StopBits = StopBits.One,
            };

            port.Open();

            while (port.IsOpen)
            {
                string str = port.ReadExisting();

                Console.WriteLine(str);
            }
        }
    }
}
