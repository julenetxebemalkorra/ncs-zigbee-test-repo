try {
    # Returns a list of all the currently available serial ports.
    [System.IO.Ports.SerialPort]::GetPortNames()

    # Replace COM20 with your actual port, and adjust baud rate and other settings as needed.
    $port = New-Object System.IO.Ports.SerialPort
    $port.PortName = "COM20"
    $port.BaudRate = 115200
    $port.Parity = [System.IO.Ports.Parity]::None
    $port.DataBits = 8
    $port.StopBits = [System.IO.Ports.StopBits]::One

    # Check if the port is not null before proceeding.
    if ($port -eq $null) {
        throw "Failed to create the SerialPort object."
    }

    # Open the connection.
    $port.Open()
    
	# zcl cmd [-d] h:dst_addr d:ep h:cluster [-p h:profile] h:cmd_ID [-l h:payload]
    # Replace "zcl cmd 0xd583 232 0x0011 -p 0x0008 0x00 -l ca0375300002ce73" with your desired command.

    $command_debug_on = "debug on"
    $command_nbr_monitor_on = "nbr monitor on"

    $command0 = "zcl cmd 0x828e 232 0x0011 -p 0x0008 0x00 -l ca0375300002ce73"
    $command1 = "zcl cmd 0xa9f2 232 0x0011 -p 0x0008 0x00 -l ca0375300002ce73"

    $port.WriteLine($command_debug_on)

    $port.WriteLine($command_nbr_monitor_on)

    # Initialize the counter.
    $successfulCommands = 0

    while ($true) {
        # Check if the port is open before writing to it.
        if ($port.IsOpen) {
            # Send the command to the device.
            $port.WriteLine($command0)
            Start-Sleep -Seconds 1
            $port.WriteLine($command1)
            $successfulCommands++
            Write-Host "Command sent successfully. Count: $($successfulCommands)"

            # Read and display incoming data.
            $receivedData = $port.ReadExisting()
            if ($receivedData -ne "") {
                Write-Host "Received data: $receivedData"
            }

        } else {
            throw "Serial port is not open."
        }

        # Wait for 10 seconds before sending the next command.
        Start-Sleep -Seconds 5
    }
}
catch {
    Write-Error "An error occurred: $_"
}
finally {
    # Close the connection when done.
    if ($port -ne $null -and $port.IsOpen) {
        $port.Close()
    }
}
