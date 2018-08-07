import sys
path = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
f = open(path, 'rb')

while True:
    x = f.read()
    sys.stdout.write(x)
