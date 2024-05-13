# Do the loop around 1,2,3

from gen_task import generate
from simulation import main_run, main_kill
import json
import time
from utils import get_dev_path, read_json_file

max_iter = 10
dev_file_path = get_dev_path()

it = 0

while it < max_iter:
    generate()
    ts = main_run()
    file_path = dev_file_path
    while True:
        d  = read_json_file(file_path + "status.json")
        if d['state'] == "finished":
            break
        time.sleep(1.5)
    
    time.sleep(1)
    main_kill(ts)
