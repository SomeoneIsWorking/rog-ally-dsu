using System.Net;
using System.Net.Sockets;
using System.Text;
using SDL3;
using SharpDX.XInput;
using static SDL3.SDL3;

class DSUServer
{
    private const int Port = 26760;
    private const int ProtocolVersion = 1001;
    private readonly Dictionary<SDL_SensorType, SDL_Sensor> sensors;
    private UdpClient udpClient;
    private IPEndPoint remoteEndPoint;
    private Controller xboxController;
    private bool isControllerConnected;

    public DSUServer()
    {
        udpClient = new UdpClient(Port);
        remoteEndPoint = new IPEndPoint(IPAddress.Any, Port);
        xboxController = new Controller(UserIndex.One);
        isControllerConnected = xboxController.IsConnected;

        // Initialize SDL
        if (SDL_Init(SDL3.SDL_InitFlags.Sensor) < 0)
        {
            throw new Exception("SDL could not initialize! SDL_Error: " + SDL_GetError());
        }
        sensors = SDL_GetSensors().ToArray().ToDictionary(SDL_GetSensorTypeForID, SDL_OpenSensor);
    }

    public void Start()
    {
        Console.WriteLine("DSU Server started...");

        while (true)
        {
            byte[] receivedData = udpClient.Receive(ref remoteEndPoint);
            HandlePacket(receivedData);
        }
    }

    private void HandlePacket(byte[] data)
    {
        if (data.Length < 16)
        {
            Console.WriteLine("Invalid packet received.");
            return;
        }

        string magic = Encoding.ASCII.GetString(data, 0, 4);
        if (magic != "DSUC")
        {
            Console.WriteLine("Invalid magic string.");
            return;
        }

        ushort version = BitConverter.ToUInt16(data, 4);
        Console.WriteLine($"Client version: {version}");
        uint messageType = BitConverter.ToUInt32(data, 16);

        switch (messageType)
        {
            case 0x100000:
                Console.WriteLine("Received protocol version request.");
                SendProtocolVersion();
                break;
            case 0x100001:
                Console.WriteLine("Received connected controllers request.");
                SendConnectedControllers();
                break;
            case 0x100002:
                Console.WriteLine("Received controller data request.");
                SendControllerData();
                break;
            default:
                Console.WriteLine("Unknown message type.");
                break;
        }
    }

    private void SendProtocolVersion()
    {
        byte[] response = new byte[20];
        Array.Copy(Encoding.ASCII.GetBytes("DSUS"), 0, response, 0, 4);
        Array.Copy(BitConverter.GetBytes((ushort)ProtocolVersion), 0, response, 4, 2);
        Array.Copy(BitConverter.GetBytes((ushort)2), 0, response, 6, 2);
        Array.Copy(BitConverter.GetBytes((uint)0), 0, response, 8, 4);
        Array.Copy(BitConverter.GetBytes((uint)1), 0, response, 12, 4);
        Array.Copy(BitConverter.GetBytes((uint)0x100000), 0, response, 16, 4);

        udpClient.Send(response, response.Length, remoteEndPoint);
    }

    private void SendConnectedControllers()
    {
        byte[] response = new byte[32];
        Array.Copy(Encoding.ASCII.GetBytes("DSUS"), 0, response, 0, 4);
        Array.Copy(BitConverter.GetBytes((ushort)ProtocolVersion), 0, response, 4, 2);
        Array.Copy(BitConverter.GetBytes((ushort)12), 0, response, 6, 2);
        Array.Copy(BitConverter.GetBytes((uint)0), 0, response, 8, 4);
        Array.Copy(BitConverter.GetBytes((uint)1), 0, response, 12, 4);
        Array.Copy(BitConverter.GetBytes((uint)0x100001), 0, response, 16, 4);

        // Example controller data
        response[20] = 0; // Slot
        response[21] = 2; // Slot state (connected)
        response[22] = 2; // Device model (full gyro)
        response[23] = 1; // Connection type (USB)
        Array.Copy(new byte[6], 0, response, 24, 6); // MAC address
        response[30] = 4; // Battery status (High)
        response[31] = 0; // Zero byte

        udpClient.Send(response, response.Length, remoteEndPoint);
    }

    private unsafe void SendControllerData()
    {
        if (!isControllerConnected)
            return;

        var state = xboxController.GetState();
        var gamepad = state.Gamepad;

        byte[] response = new byte[100];
        Array.Copy(Encoding.ASCII.GetBytes("DSUS"), 0, response, 0, 4);
        Array.Copy(BitConverter.GetBytes((ushort)ProtocolVersion), 0, response, 4, 2);
        Array.Copy(BitConverter.GetBytes((ushort)80), 0, response, 6, 2);
        Array.Copy(BitConverter.GetBytes((uint)0), 0, response, 8, 4);
        Array.Copy(BitConverter.GetBytes((uint)1), 0, response, 12, 4);
        Array.Copy(BitConverter.GetBytes((uint)0x100002), 0, response, 16, 4);

        // Example controller data
        response[20] = 0; // Slot
        response[21] = 2; // Slot state (connected)
        response[22] = 2; // Device model (full gyro)
        response[23] = 1; // Connection type (USB)
        Array.Copy(new byte[6], 0, response, 24, 6); // MAC address
        response[30] = 4; // Battery status (High)
        response[31] = 0; // Zero byte

        // Controller state data
        response[32] = 1; // Is controller connected
        Array.Copy(BitConverter.GetBytes((uint)1), 0, response, 36, 4); // Packet number
        response[40] = (byte)((gamepad.Buttons & GamepadButtonFlags.DPadLeft) != 0 ? 1 : 0); // D-Pad Left
        response[40] |= (byte)((gamepad.Buttons & GamepadButtonFlags.DPadDown) != 0 ? 2 : 0); // D-Pad Down
        response[40] |= (byte)((gamepad.Buttons & GamepadButtonFlags.DPadRight) != 0 ? 4 : 0); // D-Pad Right
        response[40] |= (byte)((gamepad.Buttons & GamepadButtonFlags.DPadUp) != 0 ? 8 : 0); // D-Pad Up
        response[40] |= (byte)((gamepad.Buttons & GamepadButtonFlags.Start) != 0 ? 16 : 0); // Start
        response[40] |= (byte)((gamepad.Buttons & GamepadButtonFlags.RightThumb) != 0 ? 32 : 0); // R3
        response[40] |= (byte)((gamepad.Buttons & GamepadButtonFlags.LeftThumb) != 0 ? 64 : 0); // L3
        response[40] |= (byte)((gamepad.Buttons & GamepadButtonFlags.Back) != 0 ? 128 : 0); // Back

        response[41] = (byte)((gamepad.Buttons & GamepadButtonFlags.Y) != 0 ? 1 : 0); // Y
        response[41] |= (byte)((gamepad.Buttons & GamepadButtonFlags.B) != 0 ? 2 : 0); // B
        response[41] |= (byte)((gamepad.Buttons & GamepadButtonFlags.A) != 0 ? 4 : 0); // A
        response[41] |= (byte)((gamepad.Buttons & GamepadButtonFlags.X) != 0 ? 8 : 0); // X
        response[41] |= (byte)((gamepad.Buttons & GamepadButtonFlags.RightShoulder) != 0 ? 16 : 0); // R1
        response[41] |= (byte)((gamepad.Buttons & GamepadButtonFlags.LeftShoulder) != 0 ? 32 : 0); // L1
        response[41] |= (byte)((gamepad.RightTrigger > 0 ? 64 : 0)); // R2
        response[41] |= (byte)((gamepad.LeftTrigger > 0 ? 128 : 0)); // L2

        response[42] = 0; // HOME button (not available on Xbox controller)
        response[43] = 0; // Touch button (not available on Xbox controller)
        response[44] = (byte)(gamepad.LeftThumbX / 256 + 128); // Left stick X
        response[45] = (byte)(gamepad.LeftThumbY / 256 + 128); // Left stick Y
        response[46] = (byte)(gamepad.RightThumbX / 256 + 128); // Right stick X
        response[47] = (byte)(gamepad.RightThumbY / 256 + 128); // Right stick Y
        response[48] = 0; // Analog D-Pad Left
        response[49] = 0; // Analog D-Pad Down
        response[50] = 0; // Analog D-Pad Right
        response[51] = 0; // Analog D-Pad Up
        response[52] = 0; // Analog Y
        response[53] = 0; // Analog B
        response[54] = 0; // Analog A
        response[55] = 0; // Analog X
        response[56] = 0; // Analog R1
        response[57] = 0; // Analog L1
        response[58] = (byte)gamepad.RightTrigger; // Analog R2
        response[59] = (byte)gamepad.LeftTrigger; // Analog L2

        // Read accelerometer and gyroscope data from SDL
        float accelX = 0, accelY = 0, accelZ = 0;
        float gyroPitch = 0, gyroYaw = 0, gyroRoll = 0;

        float* data = stackalloc float[3];
        foreach (var (type, sensor) in sensors)
        {
            if (type == SDL_SensorType.Accel)
            {
                SDL_GetSensorData(sensor, data, 3);
                accelX = data[0];
                accelY = data[1];
                accelZ = data[2];
            }
            else if (type == SDL_SensorType.Gyro)
            {
                SDL_GetSensorData(sensor, data, 3);
                gyroPitch = data[0];
                gyroYaw = data[1];
                gyroRoll = data[2];
            }
        }

        Array.Copy(BitConverter.GetBytes((ulong)0), 0, response, 72, 8); // Motion data timestamp
        Array.Copy(BitConverter.GetBytes(accelX), 0, response, 80, 4); // Accelerometer X
        Array.Copy(BitConverter.GetBytes(accelY), 0, response, 84, 4); // Accelerometer Y
        Array.Copy(BitConverter.GetBytes(accelZ), 0, response, 88, 4); // Accelerometer Z
        Array.Copy(BitConverter.GetBytes(gyroPitch), 0, response, 92, 4); // Gyroscope pitch
        Array.Copy(BitConverter.GetBytes(gyroYaw), 0, response, 96, 4); // Gyroscope yaw
        Array.Copy(BitConverter.GetBytes(gyroRoll), 0, response, 100, 4); // Gyroscope roll

        udpClient.Send(response, response.Length, remoteEndPoint);
    }
}