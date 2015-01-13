using System;
using System.Diagnostics;
using System.IO.Ports;

namespace gtrack2joy
{
    class Arduino
    {
        private bool Debug;
        public short X { get; set; }
        public short Y { get; set; }
        public short Z { get; set; }
        private SerialClient PortClient { get; set; }

        public Arduino(String portName, bool debug)
        {
            Debug = debug;
            PortClient = new SerialClient(portName, 115200);
            PortClient.OnReceiving += new EventHandler<DataStreamEventArgs>(ReceiveHandler);
            PortClient.OpenConn();
        }

        private void ReceiveHandler(object sender, DataStreamEventArgs e)
        {
            var buffer = e.Response;
            if(buffer.Length != 10) return;

            if (Debug)
            {
                foreach (var b in buffer)
                {
                    Console.Write(b);
                    Console.Write('\t');
                }

                Console.WriteLine();
            }

            X = (short) ((((UInt16) buffer[2]) << 8) | (buffer[3]) & 0xff);
            Y = (short) ((((UInt16) buffer[4]) << 8) | (buffer[5]) & 0xff);
            Z = (short) ((((UInt16) buffer[6]) << 8) | (buffer[7]) & 0xff);

            if (Debug)
            {
                Console.WriteLine(String.Format("X: {0}\tY:{1}\tZ:{2}", new object[]
                {
                    X,
                    Y,
                    Z
                }));
            }
        }

        ~Arduino()
        {
            PortClient.CloseConn();
        }
    }
}
