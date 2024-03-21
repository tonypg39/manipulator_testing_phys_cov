# Process
# Generate random parameters:
# * task_params = {} # example_dict
# * use np.random.randint() and 
# np.random.rand()*2 -1 to generate the pos_x and pos_y
# * export new dict into task_params.json in the desired location in the ros project. 
import numpy as np
import json

task_params = {
 "task_id": "a01",
 "brick_type": 2, # index 0 -> 10 defining which type from the brickDict
 "posX": -0.5, # value from -1 to 1, px is calculated as px = posX * (total_x/2)
 "posY": 1, # value from -1 to 1, py is calculated as py = posy * (total_y/2)
 "rotationZ": 45, #in degrees (-180 to 180)
 "color": 2 # index of ColorList (0 -> 9  = len(colorList))
}
file_path = "/path/to/file" #TODO: add the file location for the task_params.json

def generate():
    
    t = task_params.copy()
    t['posX'] = np.random.rand()*2 - 1
    t['posY'] = np.random.rand()*2 - 1
    t['rotationZ'] = np.random.randint(360) - 180
    t['color'] = np.random.randint(10)

    ts = json.dumps(t)

    with open(file_path, 'w') as f:
        f.write(ts)


if __name__ == "__main__":
    generate()



    