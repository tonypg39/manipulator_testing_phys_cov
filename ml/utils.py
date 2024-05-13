import json


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


