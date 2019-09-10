import json

def json_write(x, filename = "./sample.json"):
    dic = {'x': x.tolist()}
    json_str = json.dumps(dic)
    json.dump(json_str, open(filename, "w"))

def json_read(filename = "./sample.json"):
    f = open("./sample.json", "r")
    json_str = json.load(f)
    json_dict = json.loads(json_str)
    return json_dict


