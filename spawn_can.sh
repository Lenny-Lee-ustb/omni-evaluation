# ! /usr/bin/expect -f
set timeout 1              
spawn sudo ip link set can0 up type can bitrate 1000000
expect "password for roma-omni:"
send "1234\r"
send_user "\n It's OK\r"
interact