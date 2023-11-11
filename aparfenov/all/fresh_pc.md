
install docker, docker compose,
setup docker without sudo
apt-get install nvidia-container-runtime
/etc/docker/daemon.json
{
   "insecure-registries" : ["kan-rt.ddns.net:8929"]
}
docker login kan-rt.ddns.net:8929
    user: docker
    pwd: Engix_1835

gen ssh key, add to repo       # !need to replace all ssh to https
ssh-keygen, add pub to repo
git clone git://mower-project
git submodule update --init --recursive
./dc.sh build build roscore sim
./dc.sh run build
mkdir data && cd data
    wget http://kan-rt.ddns.net:18000/incoming/pidnet/PIDNet_S_ImageNet.pth.tar
    wget http://kan-rt.ddns.net:18000/incoming/pidnet/best.pt

./dc.sh --profile backend_demo --profile costmap_demo up --rosargs world:=baylands
