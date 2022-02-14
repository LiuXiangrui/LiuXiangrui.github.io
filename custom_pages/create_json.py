import json
import random
import csv 
import sys

width = 1800
height = 400

categories = ["CommonLib", "EncoderLib", "DecoderLib", "EncoderApp", "DecoderApp"]
location = []

def load_csv(csv_path: str):
    data_list = []
    with open(csv_path,'r') as csv_file:  
        data = csv.reader(csv_file)  
        for line in data:  
            data_list.append(line)  
    data_list.remove(data_list[0])

    return data_list

class FunctionDescription:
    def __init__(self, name: str, father: str, category: str, description: str, url: str) -> None:
        assert category in categories
        self.name = name
        self.father = father
        self.id = 0
        self.symbolSize = 0
        self.location = (0, 0)
        self.category = category
        self.num_childs = 0
        self.access = False
        self.url = url if url else 'http://www.baidu.com'
        self.description = description if description else "TODO"

    def serialized(self):
        data = {
            "id": str(self.id),
            "name": self.name,
            "symbolSize": str(self.symbolSize),
            "x": self.location[0],
            "y": self.location[1],
            "category": categories.index(self.category),
            "url": self.url,
            "description": str(self.description),
        }
        return data


csv_path = sys.argv[1]
data = load_csv(csv_path)

nodes = {}
for item in data:
    nodes[item[0]] = FunctionDescription(name=item[0], father=item[1], category=item[2], description=item[3], url=item[4])

links = []

count = 0
for key in nodes.keys():
    nodes[key].id = count
    count += 1

for key in nodes.keys():
    child = nodes[key]
    father_list = child.father.split(' ')
    for father in father_list:
        if father in nodes.keys():
            father = nodes[father]
            links.append({"source": str(father.id), "target": str(child.id)})
            father.num_childs += 1

nodes = sorted(nodes.items(), key=lambda x: x[1].category)
nodes = {i: j for i, j in nodes}

for key in nodes.keys():
    nodes[key].symbolSize = 20 + 1 * nodes[key].num_childs
    if nodes[key].category == "CommonLib":  # temporally set 3 categories
        nodes[key].location = (random.uniform(-width / 6, width / 6), random.uniform(-height / 2, height / 2))
    elif nodes[key].category == "EncoderLib":
        nodes[key].location = (random.uniform(-width / 2, -width / 6), random.uniform(-height / 2, 0))
    elif nodes[key].category == "EncoderApp":
        nodes[key].location = (random.uniform(-width / 2, -width / 6), random.uniform(0, height / 2))
    elif nodes[key].category == "DecoderLib":
        nodes[key].location = (random.uniform(width / 6, width / 2), random.uniform(-height / 2, 0))
    elif nodes[key].category == "DecoderApp":
        nodes[key].location = (random.uniform(width / 6, width / 2), random.uniform(0, height / 2))
    else:
        pass
 

def set_loc(key: str):
    if nodes[key].access == False:
        if nodes[key].father in nodes.keys():
            if nodes[nodes[key].father].access == False:
                set_loc(nodes[key].father)
            x = nodes[nodes[key].father].location[0]
            y = nodes[nodes[key].father].location[1]
            max_size = 7.8 * max(nodes[nodes[key].father].symbolSize, nodes[key].symbolSize)
            x = x + random.uniform(-25 - max_size, 25 + max_size)
            y = y + random.uniform(-25 - max_size, 25 + max_size)
            nodes[key].location = (x, y)
            nodes[key].access = True
        else:
            nodes[key].access = True
    return

for key in nodes.keys():
    set_loc(key)

nodes = [i.serialized() for i in nodes.values()]



data = {}
data["nodes"] = nodes
data["links"] = links
data["categories"] = [{"name": i} for i in categories]

with open("./data.json","w") as f:
    json.dump(data, f, ensure_ascii=False)

