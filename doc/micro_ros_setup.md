# MicroROS agent setup

## Connect Windows Network to WSL (Only Once)

- Open the WSL Settings app
- Go to `Networking` >> `Networking Mode` and select `Mirrored`
- Open the Windows Defender Firewall app
- Select `Advanced Settings`
- Under `Inbound Rules` add a `New Rule...`
- Select `Port` >> `UDP` >> `8888-9999`
- Keep the rest of the default settings and name it `micro-ROS UDP`
- Make sure the firewall is enabled to allow the ESP to access WSL's micro-ros

## Connect to MicroROS

In WSL, run a micro-ros agent for the ESP32's Wi-Fi Port
```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port <port> --dev <agent ip>
```
Example:
```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 --dev 172.20.10.5
```

## Debugging Tips

## View Devices on Local Network

In powershell, run

```powershell
nmap -sn <laptop_ip>/24
```
Example:
```powershell
nmap -sn 172.20.10.2/24
```
