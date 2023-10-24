echo "WID STARTING"

echo "WID 13"

echo "Checking existing node"

node --version

echo "Installing node..."

apt-get update

apt-get install -y ca-certificates curl gnupg

# mkdir -p /etc/apt/keyrings

# curl -sL https://deb.nodesource.com/setup_16.x | -E bash -

curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.5/install.sh | bash

echo "list .nvm downloaded file"

ls -a | grep .nvm

. ~/.nvm/nvm.sh

echo "Use nvm to install node"

nvm install 16

echo "Check Nodejs version"

node --version

echo "Install node modules"

cd ./approaches/checker/jslib

npm install

cd ../../..

pip install -r requirements.txt

echo "Install libc"

apt install -y libc6 libc-bin

python3 -m gunicorn app:app