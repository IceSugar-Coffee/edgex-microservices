import socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.connect(("8.8.8.8", 80))

hostIp = s.getsockname()[0]
cameraUrl = "http://"+hostIp+":8080/stream?topic=/camera/rgb/image_raw"
print(cameraUrl)