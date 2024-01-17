from time import sleep
import redis
import argparse

redis_cli = redis.Redis(host='localhost', port=6379, db=0, decode_responses=True)

parser = argparse.ArgumentParser()
parser.add_argument('-b', '--button', default="Xbox_input_button", type=str, help="name of Xbox button")
parser.add_argument('-d', '--density', default="Xbox_input_density", type=str, help="density of Xbox button")
args = parser.parse_args()

while True:
    button = redis_cli.get(args.button)
    density = redis_cli.get(args.density)
    
    if button:
        print(f"[Redis] Xbox input: {button} with density: {density}")
        # sleep(1)
    else:
        print("No Xbox input!")
        pass
