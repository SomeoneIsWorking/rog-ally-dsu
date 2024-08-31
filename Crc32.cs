
// CRC32 Implementation
using System.Security.Cryptography;

public class Crc32 : HashAlgorithm
{
    public const uint DefaultPolynomial = 0xedb88320;
    public const uint DefaultSeed = 0xffffffff;

    private uint hash;
    private uint seed;
    private readonly uint[] table;
    private static uint[]? defaultTable;

    public Crc32()
    {
        table = InitializeTable(DefaultPolynomial);
        seed = DefaultSeed;
        Initialize();
    }

    public override void Initialize()
    {
        hash = seed;
    }

    protected override void HashCore(byte[] buffer, int start, int length)
    {
        hash = CalculateHash(table, hash, buffer, start, length);
    }

    protected override byte[] HashFinal()
    {
        var hashBuffer = BitConverter.GetBytes(~hash);
        Array.Reverse(hashBuffer);
        return hashBuffer;
    }

    public override int HashSize => 32;

    public static uint Compute(byte[] buffer)
    {
        return ~CalculateHash(InitializeTable(DefaultPolynomial), DefaultSeed, buffer, 0, buffer.Length);
    }

    private static uint[] InitializeTable(uint polynomial)
    {
        if (polynomial == DefaultPolynomial && defaultTable != null)
            return defaultTable;

        var createTable = new uint[256];
        for (var i = 0; i < 256; i++)
        {
            var entry = (uint)i;
            for (var j = 0; j < 8; j++)
                if ((entry & 1) == 1)
                    entry = (entry >> 1) ^ polynomial;
                else
                    entry >>= 1;
            createTable[i] = entry;
        }

        if (polynomial == DefaultPolynomial)
            defaultTable = createTable;

        return createTable;
    }

    private static uint CalculateHash(uint[] table, uint seed, byte[] buffer, int start, int size)
    {
        var crc = seed;
        for (var i = start; i < start + size; i++)
            unchecked
            {
                crc = (crc >> 8) ^ table[buffer[i] ^ crc & 0xff];
            }
        return crc;
    }
}