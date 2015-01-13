using System;

namespace gtrack2joy
{
    class Program
    {
        private const Boolean Debug = false;

        private const string HelloText = "gtrack v 1.0 \n" +
                                         "(C) 2015 salceson & mjanczykowski\n" +
                                         "Transporting values from gtrack to joystick...";

        static void Main(string[] args)
        {
            var vjoy = new VJoy();

            vjoy.Initialize();
            vjoy.Reset();
            vjoy.Update(0);

            var portName = "COM11";

            if (args.Length >= 2)
            {
                portName = args[1];
            }

            Console.WriteLine(HelloText);

            var arduino = new Arduino(portName, Debug);

            while (true)
            {
                vjoy.SetXAxis(0, arduino.X);
                vjoy.SetYAxis(0, arduino.Y);
                vjoy.SetZAxis(0, arduino.Z);
                vjoy.Update(0);
                System.Threading.Thread.Sleep(100);
            }
        }
    }
}
