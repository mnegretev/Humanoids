#!/bin/bash

cd
git clone https://github.com/RoboCup-Humanoid-TC/GameController.git
cd GameController/ 
sudo apt update
sudo apt install -y libwebkit2gtk-4.0-dev \
    build-essential \
    curl \
    wget \
    file \
    libssl-dev \
    libgtk-3-dev \
    libayatana-appindicator3-dev \
    librsvg2-dev

curl --proto '=https' --tlsv1.2 https://sh.rustup.rs -sSf | sh
source ~/.bashrc
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.38.0/install.sh | sh
source ~/.bashrc
nvm install node
sudo apt install -y clang
cd frontend 
npm ci
npm audit fix --force
npm run build
cargo build -r
cd ..
cd game_controller_app/
cargo run -- -l