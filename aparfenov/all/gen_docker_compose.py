import os
from jinja2 import Environment, FileSystemLoader

environment = Environment(loader=FileSystemLoader("./"))
template = environment.get_template("docker-compose.yaml.jinja2")

bind_paths = []

for fn in os.listdir("ws/src"):
    rfn = os.path.realpath("ws/src/" + fn)
    bind_paths += [f"{rfn}:/cdir/ws/src/{fn}"]

content = template.render(
    bind_paths=bind_paths
    )
with open("docker-compose.yaml", mode="w", encoding="utf-8") as message:
    message.write(content)
