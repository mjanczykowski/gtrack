using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace gtrack2joy
{
    class Program
    {
        static void Main(string[] args)
        {
            var vjoy = new VJoy();

            vjoy.Initialize();
            vjoy.Reset();
            vjoy.Update(0);

            short x = 0;

            while (true)
            {
                vjoy.SetXAxis(0, x);
                vjoy.Update(0);
                System.Threading.Thread.Sleep(100);
                x--;
            }

        }
    }
}
