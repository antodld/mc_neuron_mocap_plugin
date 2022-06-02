mc_rtc plugin for neuron mocap
==

This plugin provides an access to Neuron MoCap data with an mc_rtc controller, the software Axis Neuron must be running on the same network as the controller

Installation
--

1. Build and install the project

2. Run using your [mc_rtc] interface of choice, add `mocap_plugin` to the `Plugins` configuration entry or enable the autoload option

[mc_rtc]: https://jrl-umi3218.github.io/mc_rtc/


# Installing and running Axis Neuron on Wine
--- 

1. Download Axis Neuron software : 
```bash
wget https://neuronmocap.com/sites/default/files/Axis_Neuron_x64_3_8_42_8591_20180411034619617000_X64.zip
```
2. Unzip: 
```bash 
unzip Axis_Neuron_x64_3_8_42_8591_20180411034619617000_X64.zip
```

3. Install the msi file
```bash 
wine msiexec.exe /i Axis_Neuron_x64_3_8_42_8591_20180411034619617000.msi
```

4. Plug Axis Hub and see which USB it maps to (might change depending on how much usbs are currently connected)
```bash 
ls -l /dev/serial/by-id  | grep Silicon | cut -d ">" -f 2
``` 
This returns something like '../../ttyUSB0', which is indeed '/dev/ttyUSB0'

5. Find which port it connects to in Wine
```bash
ls -l ~/.wine/dosdevices | grep /dev/ttyUSB0 | cut -d ">" -f 2
```
Shall return something (eg `~/.wine/dosdevices/com1`), otherwise see troubleshooting. 

6. Change the access write of that port 
```bash
$> sudo chmod 666 ~/.wine/dosdevices/com1 
```

7. Force-kill wine
```bash
wineserver -k 
```

8. Run and check if everything is fine in Axis Neuron
```bash
wine "C:\Program Files\NOITOM\Axis Neuron\Axis Neuron.exe" 
```
It may happens that some DLLs are missing, in that case see troubleshooting.  

# Troubleshooting
### The usb hub does not appear in Axis Neuron
1. Check if Wine sees the hub: 
```bash
ls -l ~/.wine/dosdevices/ | grep /dev/ttyUSB0
```

2. If there is no input, allow wine to see that port
   * a) Run `wine regedit`
   * b) Go to `HKEY_LOCAL_MACHINE/Wine/Ports`
   * c) Right click > New > Value chain 
   * d) Set name to COM1 and value to /dev/ttyUSB0

3. Force-kill wine and relaunch axis 
```bash 
wineserver -k && wine "C:\Program Files\NOITOM\Axis Neuron\Axis Neuron.exe" 
```

### Program fails with error 'unimplemented function msvcr120.dll'
1. Install `winetricks` 
```bash
sudo apt install winetricks
```

2. Install `vcrun2013`
```bash
winetricks -q vcrun2013
```
