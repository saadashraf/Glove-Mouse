using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO.Ports;

namespace GyroMouse
{
    /// <summary>
    /// The controller class that links serial signals to the control of mouse.
    /// It also provides methods to change mouse properties, i.e y-axis inversion, safety mode and click only mode for left mouse button.
    /// It also handles events from serial port to move the mouse pointer to desired location.
    /// </summary>
    class MouseController
    {
        public string Port { get; set; }
        public int BaudRate { get; set; }
        public bool Debug { get; set; }
        public int ClickFloor { get; set; }
        public bool SafetyMechanism { get; set; }
        public bool YInverted { get; set; }
        public bool ClickMode { get; set; }
        public MainWindow Window { get; set; }

        private SerialPort SPort;
        private string str;
        private string[] GetV = new string[5];
        private int X;
        private int Y;
        private int LMB;
        private int RMB;
        private int YInversionMultiplier = -1;
        private int YInversionVal = 65400;
        private bool LMBDown = false;
        private bool RMBDown = false;

        public MouseController() : this("COM3", 9600, false) { }
        public MouseController(string port, int baudRate) : this(port, baudRate, false) { }
        public MouseController(string port, int baudRate, bool debug)
        {
            Port = port;
            BaudRate = baudRate;
            Debug = debug;
            ClickFloor = 3;
            SafetyMechanism = false;
            YInverted = false;
            ClickMode = false;
            Init();
        }

        /// <summary>
        /// Creates a new serial port with port name and baud rate but does not open it.
        /// Binds data receive event from the serial port to data received method.
        /// </summary>
        private void Init()
        {
            SPort = new SerialPort(portName: Port, baudRate: BaudRate);
            //Console.WriteLine($"Port : {Port} Baud Rate : {BaudRate}");
            SPort.DataReceived += SPort_DataReceived;
        }

        /// <summary>
        /// This methods runs everytime some data is received in serial port.
        /// Based on the packet's leading string, it controls mouse or shows error message.
        /// </summary>
        /// <param name="sender"> Event sender </param>
        /// <param name="e"> Event argument </param>
        private void SPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            str = SPort.ReadLine();
            if (str[0] == 'G')
            {
                GetV = str.Split(' ');
                X = (int)(Convert.ToDouble(GetV[1]));
                Y = YInversionVal + YInversionMultiplier * (int)(Convert.ToDouble(GetV[2]));
                LMB = Convert.ToInt32(GetV[3]);
                RMB = Convert.ToInt32(GetV[4]);
                if (SafetyMechanism && X == 0 && Y == 0) End();
                VirtualMouse.MoveTo(X, Y);

                if (LMB <= ClickFloor && !LMBDown)
                {
                    if (ClickMode)
                    {
                        VirtualMouse.LeftClick();
                    }
                    else
                    {
                        VirtualMouse.LeftDown();
                    }
                    LMBDown = true;
                }
                else if (LMB > ClickFloor && LMBDown)
                {
                    if (!ClickMode)
                    {
                        VirtualMouse.LeftUp();
                    }
                    LMBDown = false;
                }

                if (RMB <= ClickFloor && !RMBDown)
                {
                    VirtualMouse.RightDown();
                    RMBDown = true;
                }
                else if (RMB > ClickFloor && RMBDown)
                {
                    VirtualMouse.RightUp();
                    RMBDown = false;
                }


            }
            else if (str[0] == 'M')
            {
                //str.TrimStart('M', ' ');
                //str = str.Trim();
                //Console.WriteLine(str);
            }
        }

        /// <summary>
        /// Opens the port while handling exceptions.
        /// </summary>
        /// <returns></returns>
        public bool Start()
        {
            try
            {
                SPort.Open();
                return true;
            }
            catch (Exception e)
            {
                Console.WriteLine("Exception occured while opening port");
                if (Debug)
                {
                    Console.WriteLine(e.StackTrace);
                }
                return false;
            }
        }

        /// <summary>
        /// Closes existing serial port connection.
        /// </summary>
        /// <returns></returns>
        public bool End()
        {
            try
            {
                SPort.Close();
                return true;
            }
            catch (Exception e)
            {
                Console.WriteLine("Exception occured while opening port");
                if (Debug)
                {
                    Console.WriteLine(e.StackTrace);
                }
                return false;
            }
        }

        /// <summary>
        /// Toggles Y axis movement inversion for mouse pointer movement.
        /// </summary>
        /// <returns> Inverted condition </returns>
        public bool ToggleYInversion()
        {
            if(YInverted == false)
            {
                YInverted = true;
                YInversionVal = 0;
                YInversionMultiplier = 1;
            }
            else
            {
                YInverted = false;
                YInversionVal = 65400;
                YInversionMultiplier = -1;
            }
            return YInverted;
        }

        /// <summary>
        /// Enables Y axis inversion for mouse pointer movement.
        /// </summary>
        public void EnableYInversion()
        {
            YInverted = true;
            YInversionVal = 0;
            YInversionMultiplier = 1;
        }

        /// <summary>
        /// Disables Y axis inversion for mouse pointer movement.
        /// </summary>
        public void DisableYInversion()
        {
            YInverted = false;
            YInversionVal = 65400;
            YInversionMultiplier = -1;
        }
    }
}
