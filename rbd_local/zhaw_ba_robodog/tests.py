import roslibpy

ros = roslibpy.Ros(host='localhost', port=9090)
ros.run()

print(ros.is_connected)