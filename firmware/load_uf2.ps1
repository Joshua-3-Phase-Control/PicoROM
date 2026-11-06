<#
.SYNOPSIS
Loads a UF2 into a PicoROM on Windows, with no driver to install: the board is sent to its RP2040's
USB boot mode through its reset interface (RESET_REQUEST_BOOTSEL), and the UF2 is copied to the drive
that mode shows (RPI-RP2). The board's image and settings in flash are kept (the UF2 holds firmware).

.PARAMETER Uf2
The firmware to load. Leave it out to only send the board to boot mode (or with -Backup, to save it).

.PARAMETER Board
The board's name or id (its USB serial is "id:name"), when more than one is plugged in.

.PARAMETER Backup
Saves the drive's CURRENT.UF2, the whole flash as it is (firmware, image and settings), to this file
first: copying it back later restores the board as it was.

.EXAMPLE
powershell -ExecutionPolicy Bypass -File firmware/load_uf2.ps1 -Board dme -Backup before.uf2 -Uf2 build/PicoROM-POG_27C020_70ns_MAP-2_0-map.uf2
#>
param([string]$Uf2, [string]$Board, [string]$Backup)
$ErrorActionPreference = 'Stop'

Add-Type -TypeDefinition @'
using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using Microsoft.Win32.SafeHandles;

public static class PicoRomReset
{
    // The reset interface's DeviceInterfaceGUID (firmware/usb_descriptors.c).
    static Guid ResetInterface = new Guid("e0e0e0e2-1234-5678-9abc-def012345678");

    [StructLayout(LayoutKind.Sequential)]
    struct DeviceInterfaceData { public int Size; public Guid ClassGuid; public int Flags; public IntPtr Reserved; }

    [StructLayout(LayoutKind.Sequential)]
    struct DeviceInfoData { public int Size; public Guid ClassGuid; public int DevInst; public IntPtr Reserved; }

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    struct SetupPacket { public byte RequestType; public byte Request; public ushort Value; public ushort Index; public ushort Length; }

    [DllImport("setupapi.dll", SetLastError = true, EntryPoint = "SetupDiGetClassDevsW")]
    static extern IntPtr SetupDiGetClassDevs(ref Guid guid, IntPtr enumerator, IntPtr parent, int flags);
    [DllImport("setupapi.dll", SetLastError = true)]
    static extern bool SetupDiEnumDeviceInterfaces(IntPtr set, IntPtr info, ref Guid guid, int index, ref DeviceInterfaceData data);
    [DllImport("setupapi.dll", SetLastError = true, EntryPoint = "SetupDiGetDeviceInterfaceDetailW")]
    static extern bool SetupDiGetDeviceInterfaceDetail(IntPtr set, ref DeviceInterfaceData data, IntPtr detail, int size, out int required, ref DeviceInfoData info);
    [DllImport("setupapi.dll", SetLastError = true)]
    static extern bool SetupDiDestroyDeviceInfoList(IntPtr set);
    [DllImport("cfgmgr32.dll")]
    static extern int CM_Get_Parent(out int parent, int devInst, int flags);
    [DllImport("cfgmgr32.dll", CharSet = CharSet.Unicode)]
    static extern int CM_Get_Device_ID(int devInst, char[] buffer, int length, int flags);
    [DllImport("kernel32.dll", SetLastError = true, CharSet = CharSet.Unicode, EntryPoint = "CreateFileW")]
    static extern SafeFileHandle CreateFile(string name, uint access, uint share, IntPtr security, uint creation, uint flags, IntPtr template);
    [DllImport("winusb.dll", SetLastError = true)]
    static extern bool WinUsb_Initialize(SafeFileHandle file, out IntPtr usb);
    [DllImport("winusb.dll", SetLastError = true)]
    static extern bool WinUsb_Free(IntPtr usb);
    [DllImport("winusb.dll", SetLastError = true)]
    static extern bool WinUsb_ControlTransfer(IntPtr usb, SetupPacket setup, byte[] buffer, int length, out int transferred, IntPtr overlapped);

    // Each PicoROM's reset interface: its path, and its USB serial ("id:name").
    public static List<string[]> Find()
    {
        var found = new List<string[]>();
        IntPtr set = SetupDiGetClassDevs(ref ResetInterface, IntPtr.Zero, IntPtr.Zero, 0x12);
        if (set == new IntPtr(-1)) return found;
        try
        {
            var data = new DeviceInterfaceData { Size = Marshal.SizeOf(typeof(DeviceInterfaceData)) };
            for (int i = 0; SetupDiEnumDeviceInterfaces(set, IntPtr.Zero, ref ResetInterface, i, ref data); i++)
            {
                var info = new DeviceInfoData { Size = Marshal.SizeOf(typeof(DeviceInfoData)) };
                int size;
                SetupDiGetDeviceInterfaceDetail(set, ref data, IntPtr.Zero, 0, out size, ref info);
                IntPtr detail = Marshal.AllocHGlobal(size);
                try
                {
                    Marshal.WriteInt32(detail, IntPtr.Size == 8 ? 8 : 6);
                    if (!SetupDiGetDeviceInterfaceDetail(set, ref data, detail, size, out size, ref info)) continue;
                    string path = Marshal.PtrToStringUni(detail + 4);
                    int parent;
                    var id = new char[400];
                    string serial = "";
                    if (CM_Get_Parent(out parent, info.DevInst, 0) == 0 && CM_Get_Device_ID(parent, id, id.Length, 0) == 0)
                    {
                        string device = new string(id).TrimEnd('\0');
                        serial = device.Substring(device.LastIndexOf('\\') + 1);
                    }
                    found.Add(new[] { path, serial });
                }
                finally { Marshal.FreeHGlobal(detail); }
            }
        }
        finally { SetupDiDestroyDeviceInfoList(set); }
        return found;
    }

    // RESET_REQUEST_BOOTSEL (01h), a class request to the reset interface (1), as picotool sends it (a
    // vendor request goes to the MS OS descriptor's handler): the board leaves USB for its boot mode.
    public static void Bootsel(string path)
    {
        var file = CreateFile(path, 0xC0000000, 3, IntPtr.Zero, 3, 0x40000080, IntPtr.Zero);
        if (file.IsInvalid) throw new Exception("The board's reset interface can't be opened: " + Marshal.GetLastWin32Error());
        IntPtr usb;
        if (!WinUsb_Initialize(file, out usb)) { file.Dispose(); throw new Exception("WinUSB didn't take the reset interface: " + Marshal.GetLastWin32Error()); }
        try
        {
            int sent;
            var setup = new SetupPacket { RequestType = 0x21, Request = 0x01, Value = 0, Index = 1, Length = 0 };
            WinUsb_ControlTransfer(usb, setup, null, 0, out sent, IntPtr.Zero); // it leaves USB at once: no answer
        }
        finally { WinUsb_Free(usb); file.Dispose(); }
    }
}
'@

function Find-BootDrive {
    Get-Volume | Where-Object { $_.FileSystemLabel -eq 'RPI-RP2' -and $_.DriveLetter } | Select-Object -First 1
}

function Wait-For([scriptblock]$Test, [int]$Seconds, [string]$What) {
    $until = (Get-Date).AddSeconds($Seconds)
    while ((Get-Date) -lt $until) {
        $result = & $Test
        if ($result) { return $result }
        Start-Sleep -Milliseconds 250
    }
    throw "No $What within $Seconds s"
}

$drive = Find-BootDrive
if (-not $drive) {
    $boards = @([PicoRomReset]::Find())
    if ($Board) { $boards = @($boards | Where-Object { $_[1] -split ':' -contains $Board }) }
    if ($boards.Count -ne 1) {
        $names = ([PicoRomReset]::Find() | ForEach-Object { $_[1] }) -join ', '
        throw "Found $($boards.Count) PicoROMs to send to boot mode (plugged in: $names); name one with -Board"
    }
    Write-Output "Sending $($boards[0][1]) to boot mode"
    [PicoRomReset]::Bootsel($boards[0][0])
    $drive = Wait-For { Find-BootDrive } 15 'RPI-RP2 drive'
}
$root = "$($drive.DriveLetter):\"
Write-Output "Boot drive $root ($((Get-Content "$root\INFO_UF2.TXT" | Select-Object -First 2) -join '; '))"
if ($Backup) {
    Copy-Item "$root\CURRENT.UF2" $Backup
    Write-Output "Saved the flash as it was to $Backup ($((Get-Item $Backup).Length) bytes)"
}
if ($Uf2) {
    Write-Output "Loading $Uf2"
    Copy-Item $Uf2 $root
    Wait-For { -not (Find-BootDrive) } 30 'reboot after loading' | Out-Null
    Write-Output 'Loaded: the board restarts with it'
}
