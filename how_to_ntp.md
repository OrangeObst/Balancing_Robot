# How to synchronize time with NTP

## Step 1: Configure the Pi to use the local server as an NTP source 
Edit the timesyncd configuration file: Open the configuration file in a text editor. For newer systems, this is /etc/systemd/timesyncd.conf. For older systems, it may be /etc/ntp.conf.
```
[bash]
sudo nano /etc/systemd/timesyncd.conf
```
Specify the server: In the [Time] section of the file, find the NTP= line and add the IP address or hostname of your local server. If you need to sync with a specific server, you can add it to the list of public servers.
ini
```
[Time]
NTP=your_local_server_ip_or_hostname
```
Save and exit: Save the file and exit the text editor (e.g., Ctrl+X, Y, Enter in nano). 

## Step 2: Restart the NTP service on the Pi 
Restart the service: Apply the changes by restarting the systemd-timesyncd service.
```
[bash]
sudo systemctl restart systemd-timesyncd
```
might have to manually start the service with
```
[bash]
sudo timedatectl set-ntp true
```
Check the status: Verify that the synchronization is working.
```
[bash]
timedatectl status
```
Look for System clock synchronized: yes and NTP service: active. 

## Step 3: Verify the local server is acting as an NTP server 
Ensure the local server (e.g., a Windows server) has the Windows Time service enabled and configured to allow NTP client requests.
Check that no firewall on the local server is blocking UDP port 123, which is used by NTP. 

## Step 4: Troubleshooting
If time does not sync, try manually setting the time on the Pi first to get it out of a bad state.
```
[bash]
sudo timedatectl set-time "YYYY-MM-DD HH:MM:SS"
```
Temporarily switch the Pi's DNS to a public server to get it online, then switch back to the local server once the NTP sync is established. 