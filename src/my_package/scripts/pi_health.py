import os
import subprocess

def get_cpu_temperature():
    temp = os.popen("vcgencmd measure_temp").readline()
    return temp.replace("temp=", "").strip()

def get_cpu_usage():
    load = os.getloadavg()
    return f"1 min: {load[0]}, 5 min: {load[1]}, 15 min: {load[2]}"

def get_memory_usage():
    mem_info = os.popen("free -m").readlines()[1].split()[1:4]
    total, used, free = map(int, mem_info)
    return f"Total: {total} MB, Used: {used} MB, Free: {free} MB"

def get_disk_usage():
    disk_usage = subprocess.check_output(['df', '-h', '/']).decode('utf-8').split('\n')[1]
    return disk_usage

def get_network_usage():
    network_usage = subprocess.check_output(['ifconfig']).decode('utf-8')
    return network_usage

def get_throttled_status():
    status_hex = os.popen("vcgencmd get_throttled").readline().strip().split('=')[1]
    status_bin = bin(int(status_hex, 16))[2:].zfill(19)
    
    status = {
        'Undervoltage Detected': status_bin[-1] == '1',
        'ARM Frequency Capped': status_bin[-2] == '1',
        'Currently Throttled': status_bin[-3] == '1',
        'Soft Temp Limit Active': status_bin[-4] == '1',
        'Undervoltage Has Occurred': status_bin[-17] == '1',
        'ARM Frequency Capped Has Occurred': status_bin[-18] == '1',
        'Throttling Has Occurred': status_bin[-19] == '1',
    }
    
    return status

def display_system_health():
    print("### Raspberry Pi Health Monitor ###\n")
    print(f"CPU Temperature: {get_cpu_temperature()}")
    print(f"CPU Load: {get_cpu_usage()}")
    print(f"Memory Usage: {get_memory_usage()}\n")
    
    print("Disk Usage:")
    print(get_disk_usage())
    
    print("\nNetwork Usage:")
    print(get_network_usage())
    
    throttled_status = get_throttled_status()
    print("\nThrottled Status:")
    for key, value in throttled_status.items():
        print(f"{key}: {'Yes' if value else 'No'}")
    print()

if __name__ == "__main__":
    display_system_health()
