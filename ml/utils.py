import json
import yaml

# Function to update the YAML file
def update_yaml_file(file_path, d):
    # Read the existing YAML file
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)
    
    # Add the new parameters
    # data['tuple_param'] = tuple_param
    # data['numeric_param'] = numeric_param
    
    for k,v in d.items():
        data[k] = v
        
    # Write the updated data back to the YAML file
    with open(file_path, 'w') as file:
        yaml.safe_dump(data, file)
        

def read_json_file(file_path):
    with open(file_path, 'r') as file:
        data = json.load(file)
    return data

def update_json_file(file_path,new_d):
    with open(file_path, 'r') as f:
        data = json.load(f)
    data.update(new_d)
    with open(file_path, 'w') as f:
        json.dump(data, f)

def get_dev_path():
    d = read_json_file("../config.json")
    return d['dev_path']

def get_config():
    return read_json_file("../config.json")


if __name__ == "__main__":
    d = {"noise_index": 0.9}