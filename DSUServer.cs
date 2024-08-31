using System.IO.Hashing;
using System.Net;
using System.Net.Sockets;
using System.Text;
using Ryujinx.Input.Motion.CemuHook.Protocol;
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
    public const uint Magic = 0x43555344; // DSUC
    public const ushort Version = 1001;


    private static Header GenerateHeader(int clientId)
    {
        Header header = new()
        {
            Id = (uint)clientId,
            MagicString = Magic,
            Version = Version,
            Length = 0,
        };

        return header;
    }


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
        CRC32(response);

        udpClient.Send(response, response.Length, remoteEndPoint);
    }

    private unsafe void SendControllerData()
    {
        if (!isControllerConnected)
            return; var state = xboxController.GetState();
        var gamepad = state.Gamepad;

        using var memoryStream = new MemoryStream();
        using var writer = new BinaryWriter(memoryStream);
        writer.Write(Encoding.ASCII.GetBytes("DSUS")); // Magic string
        writer.Write((ushort)ProtocolVersion); // Protocol version
        writer.Write((ushort)80); // Packet size
        writer.Write((uint)0); // Zero
        writer.Write((uint)1); // Server ID
        writer.Write((uint)0x100002); // Message type

        // Example controller data
        writer.Write((byte)0); // Slot
        writer.Write((byte)2); // Slot state (connected)
        writer.Write((byte)2); // Device model (full gyro)
        writer.Write((byte)1); // Connection type (USB)
        writer.Write(new byte[6], 0, 6); // MAC address
        writer.Write((byte)4); // Battery status (High)

        // Controller state data
        writer.Write((byte)1); // Is controller connected
        writer.Write(BitConverter.GetBytes((uint)1)); // Packet number
        byte buttons = 0;
        buttons |= (byte)((gamepad.Buttons & GamepadButtonFlags.DPadLeft) != 0 ? 1 : 0); // D-Pad Left
        buttons |= (byte)((gamepad.Buttons & GamepadButtonFlags.DPadDown) != 0 ? 2 : 0); // D-Pad Down
        buttons |= (byte)((gamepad.Buttons & GamepadButtonFlags.DPadRight) != 0 ? 4 : 0); // D-Pad Right
        buttons |= (byte)((gamepad.Buttons & GamepadButtonFlags.DPadUp) != 0 ? 8 : 0); // D-Pad Up
        buttons |= (byte)((gamepad.Buttons & GamepadButtonFlags.Start) != 0 ? 16 : 0); // Start
        buttons |= (byte)((gamepad.Buttons & GamepadButtonFlags.RightThumb) != 0 ? 32 : 0); // R3
        buttons |= (byte)((gamepad.Buttons & GamepadButtonFlags.LeftThumb) != 0 ? 64 : 0); // L3
        buttons |= (byte)((gamepad.Buttons & GamepadButtonFlags.Back) != 0 ? 128 : 0); // Back

        writer.Write(buttons);

        byte buttons2 = 0;
        buttons2 |= (byte)((gamepad.Buttons & GamepadButtonFlags.Y) != 0 ? 1 : 0); // Y
        buttons2 |= (byte)((gamepad.Buttons & GamepadButtonFlags.B) != 0 ? 2 : 0); // B
        buttons2 |= (byte)((gamepad.Buttons & GamepadButtonFlags.A) != 0 ? 4 : 0); // A
        buttons2 |= (byte)((gamepad.Buttons & GamepadButtonFlags.X) != 0 ? 8 : 0); // X
        buttons2 |= (byte)((gamepad.Buttons & GamepadButtonFlags.RightShoulder) != 0 ? 16 : 0); // R1
        buttons2 |= (byte)((gamepad.Buttons & GamepadButtonFlags.LeftShoulder) != 0 ? 32 : 0); // L1
        buttons2 |= (byte)(gamepad.RightTrigger > 0 ? 64 : 0); // R2
        buttons2 |= (byte)(gamepad.LeftTrigger > 0 ? 128 : 0); // L2

        writer.Write(buttons2);

        writer.Write((byte)0); // HOME button (not available on Xbox controller)
        writer.Write((byte)0); // Touch button (not available on Xbox controller)
        writer.Write((byte)(gamepad.LeftThumbX / 256 + 128)); // Left stick X
        writer.Write((byte)(gamepad.LeftThumbY / 256 + 128)); // Left stick Y
        writer.Write((byte)(gamepad.RightThumbX / 256 + 128)); // Right stick X
        writer.Write((byte)(gamepad.RightThumbY / 256 + 128)); // Right stick Y
        writer.Write((byte)0); // Analog D-Pad Left
        writer.Write((byte)0); // Analog D-Pad Down
        writer.Write((byte)0); // Analog D-Pad Right
        writer.Write((byte)0); // Analog D-Pad Up
        writer.Write((byte)0); // Analog Y
        writer.Write((byte)0); // Analog B
        writer.Write((byte)0); // Analog A
        writer.Write((byte)0); // Analog X
        writer.Write((byte)0); // Analog R1
        writer.Write((byte)0); // Analog L1
        writer.Write((byte)gamepad.RightTrigger); // Analog R2
        writer.Write((byte)gamepad.LeftTrigger); // Analog L2

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

        writer.Write(BitConverter.GetBytes((ulong)DateTimeOffset.UtcNow.ToUnixTimeMilliseconds())); // Motion data timestamp
        writer.Write(BitConverter.GetBytes(accelX)); // Accelerometer X
        writer.Write(BitConverter.GetBytes(accelY)); // Accelerometer Y
        writer.Write(BitConverter.GetBytes(accelZ)); // Accelerometer Z
        writer.Write(BitConverter.GetBytes(gyroPitch)); // Gyroscope pitch
        writer.Write(BitConverter.GetBytes(gyroYaw)); // Gyroscope yaw
        writer.Write(BitConverter.GetBytes(gyroRoll)); // Gyroscope roll
        Console.WriteLine($"Accel: {accelX}, {accelY}, {accelZ}, Gyro: {gyroPitch}, {gyroYaw}, {gyroRoll}");
        writer.Flush();
        byte[] response = memoryStream.ToArray();
        CRC32(response);

        udpClient.Send(response, response.Length, remoteEndPoint);
    }

    private static unsafe void CRC32(byte[] response)
    {
        // Calculate CRC32
        var crc32 = Crc32.Hash(response);

        // Replace offset 8 to 12 with CRC32
        Array.Copy(crc32, 0, response, 8, 4);
    }

}