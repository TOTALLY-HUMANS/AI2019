from wireless import Wireless
import json
import sys

config = sys.argv[1]
print(config)
with open(config, 'r') as f:
	data = json.load(f)
print(data)
